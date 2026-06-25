"""Fail-fast invariants for semantic geometry plans."""

from __future__ import annotations

from semantic_geometry_builder.models import (
    ROUTE_ALLOWED_REPRESENTATIONS,
    ConstructionBodyPlanRecord,
    CutHostOperationRecord,
    GeometryBuildInput,
    InterfacePlanRecord,
    RouteLiteral,
    SemanticEntitySpec,
    SurfacePartitionRecord,
    SurfacePlanRecord,
    TagPlanRecord,
    VolumePlanRecord,
)


def validate_geometry_input(build_input: GeometryBuildInput) -> GeometryBuildInput:
    """Validate adapter-normalized input before route-aware planning."""
    errors: list[str] = []
    polygon_ids: set[str] = set()
    entity_ids: set[str] = set()

    for polygon in build_input.polygons:
        if not polygon.polygon_id:
            errors.append("polygon_id must be non-empty")
        elif polygon.polygon_id in polygon_ids:
            errors.append(f"duplicate polygon_id: {polygon.polygon_id}")
        polygon_ids.add(polygon.polygon_id)
        if polygon.exterior and len(polygon.exterior) < 3:
            errors.append(f"{polygon.polygon_id} exterior requires at least 3 points")

    for entity in build_input.entities:
        if not entity.semantic_id:
            errors.append("semantic_id must be non-empty")
        elif entity.semantic_id in entity_ids:
            errors.append(f"duplicate semantic_id: {entity.semantic_id}")
        entity_ids.add(entity.semantic_id)
        if not entity.material_id:
            errors.append(f"{entity.semantic_id} material_id must be non-empty")
        for polygon_id in entity.polygon_ids:
            if polygon_id not in polygon_ids:
                errors.append(
                    f"{entity.semantic_id} references unknown polygon_id: {polygon_id}"
                )

    if not any(_is_air_like_solution_entity(entity) for entity in build_input.entities):
        errors.append("GeometryBuildInput requires an air/vacuum solution entity")

    if errors:
        raise ValueError("Invalid GeometryBuildInput: " + "; ".join(errors))
    return build_input


def validate_selected_route(
    build_input: GeometryBuildInput,
    route: RouteLiteral,
) -> None:
    """Validate that every conductor has an explicit representation for route."""
    allowed = ROUTE_ALLOWED_REPRESENTATIONS.get(route)
    if allowed is None:
        raise ValueError(f"Unsupported route: {route}")

    errors: list[str] = []
    for entity in build_input.entities:
        if not _requires_route_representation(entity):
            continue
        representation = entity.route_representations.get(route)
        if representation is None:
            errors.append(f"{entity.semantic_id} does not define Route {route}")
        elif representation not in allowed:
            errors.append(
                f"{entity.semantic_id} Route {route} representation "
                f"{representation!r} is not supported"
            )
    if errors:
        raise ValueError("Invalid selected route: " + "; ".join(errors))


def validate_route_operation_coverage(
    *,
    construction_bodies: tuple[ConstructionBodyPlanRecord, ...],
    cut_operations: tuple[CutHostOperationRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Validate Route A/B construction-body, cut, and shell-surface linkage."""
    surface_ids = {surface.surface_id for surface in surfaces}
    body_ids = {body.construction_body_id for body in construction_bodies}
    operation_body_ids = {
        construction_body_id
        for operation in cut_operations
        for construction_body_id in operation.construction_body_ids
    }
    operation_surface_ids = {
        surface_id
        for operation in cut_operations
        for surface_id in operation.exposed_surface_ids
    }

    errors: list[str] = []
    for body in construction_bodies:
        missing_surfaces = [
            surface_id
            for surface_id in body.expected_surface_ids
            if surface_id not in surface_ids
        ]
        if missing_surfaces:
            errors.append(
                f"{body.construction_body_id} expects missing shell surfaces "
                f"{missing_surfaces!r}"
            )

    missing_operation_bodies = sorted(body_ids - operation_body_ids)
    if missing_operation_bodies:
        errors.append(
            f"construction bodies without cut operations {missing_operation_bodies!r}"
        )

    missing_operation_surfaces = sorted(operation_surface_ids - surface_ids)
    if missing_operation_surfaces:
        errors.append(
            f"cut operations expose missing surfaces {missing_operation_surfaces!r}"
        )

    if errors:
        raise ValueError("Invalid route operation plan: " + "; ".join(errors))


def validate_surface_sheet_interface_coverage(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Fail if Route A sheet conductors are not represented by interfaces."""
    sheet_ids = {
        entity.semantic_id
        for entity in build_input.entities
        if not _is_solution_entity(entity)
        and entity.route_representations.get(route) == "surface_sheet"
    }
    if not sheet_ids:
        return

    covered_ids: set[str] = set()
    for surface in surfaces:
        if surface.interface_id is None:
            continue
        owner_ids = surface.metadata.get(
            "owner_semantic_ids",
            (surface.owner_semantic_id,),
        )
        if isinstance(owner_ids, str):
            owner_ids = (owner_ids,)
        covered_ids.update(str(owner_id) for owner_id in owner_ids)

    missing = sorted(sheet_ids - covered_ids)
    if missing:
        raise ValueError(
            "surface_sheet entities require planned interface surface coverage: "
            f"{missing!r}"
        )


def validate_route_volume_surface_refs(
    *,
    route: RouteLiteral,
    volumes: tuple[VolumePlanRecord, ...],
) -> None:
    """Fail fast when Route C retained volumes have no planned boundaries."""
    if route != "C":
        return
    missing = sorted(
        volume.volume_id
        for volume in volumes
        if not volume.surface_refs
    )
    if missing:
        raise ValueError(
            "Route C material volumes require planned boundary surfaces: "
            f"{missing!r}"
        )


def validate_surface_partition_coverage(
    *,
    interfaces: tuple[InterfacePlanRecord, ...],
    surface_partitions: tuple[SurfacePartitionRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Validate that partition records point to real parent/child plan ids."""
    interface_ids = {interface.interface_id for interface in interfaces}
    surfaces_by_id = {surface.surface_id: surface for surface in surfaces}
    errors: list[str] = []
    partitions_by_parent: dict[str, list[SurfacePartitionRecord]] = {}
    for partition in surface_partitions:
        partitions_by_parent.setdefault(
            partition.parent_interface_id,
            [],
        ).append(partition)
        if partition.parent_interface_id not in interface_ids:
            errors.append(
                f"{partition.partition_id} references unknown parent interface "
                f"{partition.parent_interface_id}"
            )
        child_surface = surfaces_by_id.get(partition.child_surface_id)
        if child_surface is None:
            errors.append(
                f"{partition.partition_id} references unknown child surface "
                f"{partition.child_surface_id}"
            )
            continue
        geometry_ref = child_surface.geometry_ref
        if "outer_loop" not in geometry_ref and "loop_geometry_ref" not in geometry_ref:
            errors.append(
                f"{partition.partition_id} child surface lacks loop geometry"
            )

    for parent_interface_id, partitions in partitions_by_parent.items():
        if len(partitions) <= 1:
            continue
        core_partitions = [
            partition
            for partition in partitions
            if partition.band_max_um is None
        ]
        band_partitions = [
            partition
            for partition in partitions
            if partition.band_max_um is not None
        ]
        if len(core_partitions) != 1:
            errors.append(
                f"{parent_interface_id} inset children require exactly one core"
            )
            continue
        expected_min = 0.0
        for partition in sorted(
            band_partitions,
            key=lambda item: item.band_min_um if item.band_min_um is not None else -1.0,
        ):
            if partition.band_min_um is None:
                errors.append(f"{partition.partition_id} band_min_um is required")
                continue
            if partition.band_min_um != expected_min:
                errors.append(
                    f"{partition.partition_id} starts at {partition.band_min_um}, "
                    f"expected {expected_min}"
                )
            if (
                partition.band_max_um is None
                or partition.band_max_um <= partition.band_min_um
            ):
                errors.append(f"{partition.partition_id} has invalid band range")
                continue
            expected_min = partition.band_max_um
        core_min = core_partitions[0].band_min_um
        if core_min is not None and core_min != expected_min:
            errors.append(
                f"{core_partitions[0].partition_id} starts at {core_min}, "
                f"expected {expected_min}"
            )
    if errors:
        raise ValueError("Invalid surface partition plan: " + "; ".join(errors))


def validate_tag_plan_coverage(
    *,
    surfaces: tuple[SurfacePlanRecord, ...],
    volumes: tuple[VolumePlanRecord, ...],
    tags: tuple[TagPlanRecord, ...],
) -> None:
    """Validate that every backend-live geometry record has a tag plan.

    Backend construction may create construction-only helper surfaces or
    volumes, but anything live in the final geometry must have a `TagPlanRecord`
    before OCC assigns dim-tags. This keeps physical groups a deterministic
    projection of the plan rather than a backend afterthought.
    """
    live_sources = {
        ("surface", surface.surface_id)
        for surface in surfaces
        if not surface.construction_only
    }
    live_sources.update(
        ("volume", volume.volume_id)
        for volume in volumes
        if not volume.construction_only
    )
    tag_sources = {
        (tag.source_record_kind, tag.source_record_id)
        for tag in tags
    }

    missing = sorted(live_sources - tag_sources)
    extra = sorted(tag_sources - live_sources)
    if missing or extra:
        parts = []
        if missing:
            parts.append(f"missing tags for {missing!r}")
        if extra:
            parts.append(f"tags reference non-live records {extra!r}")
        raise ValueError("Invalid tag plan coverage: " + "; ".join(parts))


def _is_solution_entity(entity: SemanticEntitySpec) -> bool:
    tokens = {
        *entity.role.lower().replace("_", " ").replace("-", " ").split(),
        entity.geometry_kind.lower().replace("-", "_"),
    }
    return bool(tokens & {"solution", "air", "substrate", "dielectric", "domain"})


def _is_air_like_solution_entity(entity: SemanticEntitySpec) -> bool:
    if not _is_solution_entity(entity):
        return False
    tokens = {
        *str(entity.semantic_id).lower().replace("_", " ").split(),
        *str(entity.role).lower().replace("_", " ").split(),
        *str(entity.geometry_kind).lower().replace("_", " ").split(),
        *str(entity.material_id).lower().replace("_", " ").split(),
    }
    return bool(tokens & {"air", "vacuum"})


def _requires_route_representation(entity: SemanticEntitySpec) -> bool:
    if _is_solution_entity(entity):
        return False
    if entity.route_representations:
        return True
    if entity.part_role is not None:
        return True
    role_tokens = set(str(entity.role).lower().replace("_", " ").split())
    return bool(role_tokens & {"metal", "conductor"})


