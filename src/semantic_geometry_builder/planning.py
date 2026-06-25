"""Route-first construction planning for semantic geometry v1.

This module owns semantic planning only. It recognizes interface intent, expands
inset children, plans Route A/B construction cuts, Route C retained volumes, and
physical tag intent before any backend entity tag exists.

Route special cases to preserve during implementation:

- Indium bump to ground-plane contact is not a fake ground-volume void. The
  route plan must recognize the XY contact patch before OCC creation, split the
  affected ground face into contact patch plus remainder surface, and make both
  touching volumes/shells reference the same live contact surface when the
  selected route needs retained material or PEC shell geometry.
- Inset interfaces are not overlay masks. The route plan must replace a parent
  interface with child ring/core `SurfacePlanRecord`s before backend lowering.
  The child surfaces must exactly cover the parent interface without overlap or
  gaps, and feasibility must be checked before treating the route as buildable.

Route outputs:

- Route A: interface-owned `surface_sheet` conductors for face metal and
  airbridge decks, plus `cutout_boundary_shell` PEC surfaces for bumps/posts.
- Route B: `cutout_boundary_shell` PEC surfaces from construction bodies.
- Route C: retained `material_volume` records assembled from planned shared
  surfaces.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from semantic_geometry_builder.models import (
    ConstructionBodyPlanRecord,
    ConstructionPlanRecord,
    CutHostOperationRecord,
    GeometryBuildInput,
    InterfacePlanRecord,
    RouteLiteral,
    SemanticEntitySpec,
    SurfacePartitionRecord,
    SurfacePlanRecord,
    SurfaceRefRecord,
    TagPlanRecord,
    VolumePlanRecord,
)
from semantic_geometry_builder.validation import (
    _is_air_like_solution_entity,
    _is_solution_entity,
    validate_route_operation_coverage,
    validate_route_volume_surface_refs,
    validate_selected_route,
    validate_surface_partition_coverage,
    validate_surface_sheet_interface_coverage,
    validate_tag_plan_coverage,
)

_GEOMETRY_REF_METADATA_KEYS = (
    "plane",
    "contact_plane",
    "footprint",
    "outer_loop",
    "hole_loops",
    "loop_geometry_ref",
    "inset_band",
)
_INTERFACE_KIND_ORDER = ("MM", "SS", "AA", "MS", "MA", "SA")


def build_route_construction_plan(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
) -> ConstructionPlanRecord:
    """Build the route-aware plan consumed by bottom-up OCC construction.

    The plan is the semantic center of v1. Interface recognition, ring
    partitioning, surface ownership, and tag ownership are decided before OCC
    geometry exists.

    The backend must be able to build from this record without global
    `occ.fragment()`: surfaces carry loop geometry, volumes reference those
    surfaces, Route A/B cuts are explicit `CutHostOperationRecord`s, and
    `TagPlanRecord`s define the physical names before dim-tags exist.

    Tags are planned before backend construction. Every backend-live
    `SurfacePlanRecord` or `VolumePlanRecord` must either get a `TagPlanRecord`
    or be explicitly marked `construction_only`.
    """
    validate_selected_route(build_input, route)
    interfaces = recognize_route_interfaces(build_input, route=route)
    surface_partitions = plan_surface_partitions(
        build_input,
        route=route,
        interfaces=interfaces,
    )
    construction_bodies = plan_route_construction_bodies(
        build_input,
        route=route,
    )
    surfaces = plan_route_surfaces(
        build_input,
        route=route,
        interfaces=interfaces,
        surface_partitions=surface_partitions,
        construction_bodies=construction_bodies,
    )
    validate_surface_sheet_interface_coverage(
        build_input,
        route=route,
        surfaces=surfaces,
    )
    volumes = plan_route_volumes(build_input, route=route, surfaces=surfaces)
    validate_route_volume_surface_refs(route=route, volumes=volumes)
    cut_operations = plan_cut_host_operations(
        route=route,
        construction_bodies=construction_bodies,
    )
    tags = plan_route_tags(route=route, surfaces=surfaces, volumes=volumes)
    validate_route_operation_coverage(
        construction_bodies=construction_bodies,
        cut_operations=cut_operations,
        surfaces=surfaces,
    )
    validate_surface_partition_coverage(
        interfaces=interfaces,
        surface_partitions=surface_partitions,
        surfaces=surfaces,
    )
    validate_tag_plan_coverage(surfaces=surfaces, volumes=volumes, tags=tags)
    return ConstructionPlanRecord(
        route=route,
        interfaces=interfaces,
        surface_partitions=surface_partitions,
        surfaces=surfaces,
        volumes=volumes,
        construction_bodies=construction_bodies,
        cut_operations=cut_operations,
        tags=tags,
        metadata={
            "backend_strategy": "surface_plan_first_bottom_up_occ",
            "fragment_first_disabled": True,
        },
    )


def recognize_route_interfaces(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
) -> tuple[InterfacePlanRecord, ...]:
    """Recognize interfaces before creating OCC geometry.

    Route-specific recognizers should own each interface rule: draw/ground
    shared edges, XY footprint overlap contacts, stack domain boundaries, ports,
    and exposed EM boundaries. Current fixture metadata may seed those rules
    through `interface_intents_2d`, including generic `interfaces` entries with
    explicit `kind`.
    """
    del route
    intents = dict(build_input.metadata.get("interface_intents_2d", {}))
    records: list[InterfacePlanRecord] = []
    for index, intent in enumerate(intents.get("interfaces", ())):
        if not isinstance(intent, Mapping):
            raise ValueError("interfaces entries must be mappings")
        kind = str(intent.get("kind", ""))
        if kind not in _INTERFACE_KIND_ORDER:
            raise ValueError(f"invalid interface kind: {kind!r}")
        owners = tuple(str(owner) for owner in intent.get("owner_semantic_ids", ()))
        if len(owners) != 2:
            raise ValueError(f"invalid interface owners: {intent!r}")
        records.append(
            InterfacePlanRecord(
                interface_id=str(
                    intent.get("interface_id") or f"{kind}__INTENT__{index:04d}"
                ),
                kind=kind,
                owner_semantic_ids=(owners[0], owners[1]),
                recognition_rule=str(intent.get("recognition_rule", "explicit")),
                source_polygon_ids=tuple(
                    str(value)
                    for value in intent.get("source_polygon_ids", ())
                ),
                metadata=dict(intent),
            )
        )
    for index, intent in enumerate(intents.get("metal_metal_contact_edges", ())):
        if not isinstance(intent, Mapping):
            raise ValueError("metal_metal_contact_edges entries must be mappings")
        owners = tuple(str(owner) for owner in intent.get("owner_semantic_ids", ()))
        if len(owners) != 2:
            raise ValueError(f"invalid MM edge intent owners: {intent!r}")
        records.append(
            InterfacePlanRecord(
                interface_id=f"MM__CONTACT_EDGE__{index:04d}",
                kind="MM",
                owner_semantic_ids=(owners[0], owners[1]),
                recognition_rule="draw_edge_overlaps_ground_mask_cutout_edge",
                source_polygon_ids=tuple(
                    str(intent[key])
                    for key in ("source_polygon_id", "ground_polygon_id")
                    if key in intent
                ),
                metadata=dict(intent),
            )
        )
    for index, intent in enumerate(intents.get("metal_ground_contact_patches", ())):
        if not isinstance(intent, Mapping):
            raise ValueError("metal_ground_contact_patches entries must be mappings")
        owners = tuple(str(owner) for owner in intent.get("owner_semantic_ids", ()))
        if len(owners) != 2:
            raise ValueError(f"invalid contact patch intent owners: {intent!r}")
        records.append(
            InterfacePlanRecord(
                interface_id=f"MM__CONTACT_PATCH__{index:04d}",
                kind="MM",
                owner_semantic_ids=(owners[0], owners[1]),
                recognition_rule="projected_xy_footprint_overlap",
                source_polygon_ids=tuple(
                    str(intent[key])
                    for key in ("source_polygon_id", "ground_polygon_id")
                    if key in intent
                ),
                metadata=dict(intent),
            )
        )
    return tuple(records)


def plan_surface_partitions(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interfaces: tuple[InterfacePlanRecord, ...],
) -> tuple[SurfacePartitionRecord, ...]:
    """Plan parent-interface partitions before live surfaces are created.

    Ring/core regions are partition intent, not backend geometry. Each returned
    `SurfacePartitionRecord` must point to a child `SurfacePlanRecord` that
    will be created directly by the backend. The parent interface itself is a
    semantic aggregate and must not be cut by OCC after creation.
    """
    interface_ids = {interface.interface_id for interface in interfaces}
    raw_partitions = build_input.metadata.get("surface_partitions", ())
    if raw_partitions in (None, ()):
        return ()
    if not isinstance(raw_partitions, tuple | list):
        raise TypeError("surface_partitions metadata must be a sequence")

    records: list[SurfacePartitionRecord] = []
    for index, intent in enumerate(raw_partitions):
        if not isinstance(intent, Mapping):
            raise TypeError("surface_partitions entries must be mappings")
        valid_routes = tuple(
            str(value)
            for value in intent.get("valid_routes", (route,))
        )
        if route not in valid_routes:
            continue
        parent_interface_id = str(intent.get("parent_interface_id", ""))
        if parent_interface_id not in interface_ids:
            raise ValueError(
                f"surface partition references unknown interface "
                f"{parent_interface_id!r}"
            )
        label = str(intent.get("label", "")).strip()
        if not label:
            raise ValueError("surface partition label must be non-empty")
        child_surface_id = str(
            intent.get("child_surface_id")
            or f"SURF__{parent_interface_id}__{label}"
        )
        band_min_um = intent.get("band_min_um")
        band_max_um = intent.get("band_max_um")
        metadata = dict(intent)
        metadata.setdefault(
            "inset_band",
            {
                "label": label,
                "min_um": band_min_um,
                "max_um": band_max_um,
            },
        )
        records.append(
            SurfacePartitionRecord(
                partition_id=str(
                    intent.get("partition_id")
                    or f"PART__{parent_interface_id}__{index:04d}"
                ),
                parent_interface_id=parent_interface_id,
                child_surface_id=child_surface_id,
                label=label,
                band_min_um=(
                    float(band_min_um) if band_min_um is not None else None
                ),
                band_max_um=(
                    float(band_max_um) if band_max_um is not None else None
                ),
                parameterization=str(
                    intent.get("parameterization", "planar_uv")
                ),
                valid_routes=valid_routes,
                metadata=metadata,
            )
        )
    return tuple(records)


def plan_route_construction_bodies(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
) -> tuple[ConstructionBodyPlanRecord, ...]:
    """Plan Route A/B cutter bodies without making them final geometry.

    Route A only gives construction bodies to conductors represented as
    `cutout_boundary_shell` such as bumps/posts. Route A `surface_sheet`
    conductors are not cutters. Route B gives every `cutout_boundary_shell`
    conductor a construction body. Route C has no construction bodies because
    material volumes survive.
    """
    if route == "C":
        return ()

    default_host = _default_host_solution_id(build_input)
    records: list[ConstructionBodyPlanRecord] = []
    for entity in build_input.entities:
        if _is_solution_entity(entity):
            continue
        representation = entity.route_representations.get(route)
        if representation != "cutout_boundary_shell":
            continue

        host_id = entity.host_void_semantic_id or default_host
        surface_id = f"SURF__{route}__SHELL__{entity.semantic_id}"
        records.append(
            ConstructionBodyPlanRecord(
                construction_body_id=f"CBODY__{route}__{entity.semantic_id}",
                owner_semantic_id=entity.semantic_id,
                host_semantic_id=host_id,
                representation=representation,
                geometry_ref=_entity_geometry_ref(
                    entity,
                    representation=representation,
                ),
                expected_surface_ids=(surface_id,),
                valid_routes=(route,),
            )
        )
    return tuple(records)


def plan_route_surfaces(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interfaces: tuple[InterfacePlanRecord, ...],
    surface_partitions: tuple[SurfacePartitionRecord, ...],
    construction_bodies: tuple[ConstructionBodyPlanRecord, ...],
) -> tuple[SurfacePlanRecord, ...]:
    """Plan route-specific surfaces without building geometry.

    Route A should plan thin sheet interfaces and PEC shell/contact surfaces. Route B
    should plan host-owned cutout shell surfaces. Route C should plan retained
    material top/bottom/sidewall/contact surfaces.

    Partitioned interfaces must already be represented as child live surfaces
    in this stage. The backend should only receive surfaces it can build
    directly from point/curve/loop metadata.

    Route A `surface_sheet` conductors are not standalone surfaces here. They
    must appear as interface-owned `MS`, `MA`, `MM`, or `SA` surfaces, so metal
    coverage replaces the bare substrate-air interface instead of overlapping it.
    """
    partitions_by_interface: dict[str, list[SurfacePartitionRecord]] = {}
    for partition in surface_partitions:
        partitions_by_interface.setdefault(
            partition.parent_interface_id,
            [],
        ).append(partition)

    records: list[SurfacePlanRecord] = []
    for interface in interfaces:
        geometry_ref = {
            "from_interface_id": interface.interface_id,
            **_geometry_ref_from_metadata(interface.metadata),
        }
        partitions = partitions_by_interface.get(interface.interface_id, ())
        if partitions:
            parent_surface_id = f"SURF__{interface.interface_id}"
            for partition in partitions:
                child_geometry_ref = {
                    **geometry_ref,
                    "partition_id": partition.partition_id,
                    "parent_interface_id": partition.parent_interface_id,
                    **_geometry_ref_from_metadata(partition.metadata),
                }
                records.append(
                    SurfacePlanRecord(
                        surface_id=partition.child_surface_id,
                        owner_semantic_id=interface.owner_semantic_ids[0],
                        surface_role=f"{route}_planned_interface_partition",
                        geometry_ref=child_geometry_ref,
                        interface_id=interface.interface_id,
                        parent_surface_id=parent_surface_id,
                        partition_label=partition.label,
                        solver_use=interface.solver_use or "solver_active",
                        valid_routes=(route,),
                        metadata={
                            "interface_kinds": _interface_kinds(interface),
                            "owner_semantic_ids": interface.owner_semantic_ids,
                        },
                    )
                )
            continue
        records.append(
            SurfacePlanRecord(
                surface_id=f"SURF__{interface.interface_id}",
                owner_semantic_id=interface.owner_semantic_ids[0],
                surface_role=f"{route}_planned_interface",
                geometry_ref=geometry_ref,
                interface_id=interface.interface_id,
                solver_use=interface.solver_use or "solver_active",
                valid_routes=(route,),
                metadata={
                    "interface_kinds": _interface_kinds(interface),
                    "owner_semantic_ids": interface.owner_semantic_ids,
                },
            )
        )

    construction_body_by_surface_id = {
        surface_id: body
        for body in construction_bodies
        for surface_id in body.expected_surface_ids
    }
    for entity in build_input.entities:
        if _is_solution_entity(entity):
            continue
        representation = entity.route_representations.get(route)
        if representation == "cutout_boundary_shell":
            surface_id = f"SURF__{route}__SHELL__{entity.semantic_id}"
            body = construction_body_by_surface_id.get(surface_id)
            records.append(
                SurfacePlanRecord(
                    surface_id=surface_id,
                    owner_semantic_id=entity.semantic_id,
                    surface_role="cutout_boundary_shell",
                    geometry_ref={
                        **_entity_geometry_ref(
                            entity,
                            representation=representation,
                        ),
                        "construction_body_id": (
                            body.construction_body_id if body is not None else None
                        ),
                    },
                    valid_routes=(route,),
                    solver_use="solver_active",
                )
            )
    return tuple(records)


def plan_route_volumes(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> tuple[VolumePlanRecord, ...]:
    """Plan retained or construction-only volumes from planned surfaces."""
    surface_ids_by_owner: dict[str, list[str]] = {}
    for surface in surfaces:
        owner_ids = surface.metadata.get(
            "owner_semantic_ids",
            (surface.owner_semantic_id,),
        )
        if isinstance(owner_ids, str):
            owner_ids = (owner_ids,)
        for owner_id in owner_ids:
            surface_ids_by_owner.setdefault(str(owner_id), []).append(
                surface.surface_id
            )

    records: list[VolumePlanRecord] = []
    for entity in build_input.entities:
        if _is_solution_entity(entity):
            continue
        representation = entity.route_representations.get(route)
        if route == "C" and representation == "material_volume":
            records.append(
                VolumePlanRecord(
                    volume_id=f"VOL__{entity.semantic_id}",
                    owner_semantic_id=entity.semantic_id,
                    material_id=entity.material_id,
                    surface_refs=tuple(
                        SurfaceRefRecord(
                            surface_id=surface_id,
                            orientation="forward",
                            role="planned_boundary",
                        )
                        for surface_id in surface_ids_by_owner.get(
                            entity.semantic_id,
                            (),
                        )
                    ),
                    valid_routes=(route,),
                    metadata={"representation": representation},
                )
            )
    return tuple(records)


def plan_cut_host_operations(
    *,
    route: RouteLiteral,
    construction_bodies: tuple[ConstructionBodyPlanRecord, ...],
) -> tuple[CutHostOperationRecord, ...]:
    """Group Route A/B construction bodies into host-cut operation plans.

    This is semantic grouping only. The backend may execute compatible records
    in batches, but must recover every member construction body and exposed
    shell surface from these operation ids.
    """
    if route == "C":
        return ()

    bodies_by_host: dict[str, list[ConstructionBodyPlanRecord]] = {}
    for body in construction_bodies:
        bodies_by_host.setdefault(body.host_semantic_id, []).append(body)

    return tuple(
        CutHostOperationRecord(
            operation_id=f"CUT__{route}__{host_id}",
            host_semantic_id=host_id,
            construction_body_ids=tuple(
                body.construction_body_id for body in bodies
            ),
            exposed_surface_ids=tuple(
                surface_id
                for body in bodies
                for surface_id in body.expected_surface_ids
            ),
            valid_routes=(route,),
        )
        for host_id, bodies in bodies_by_host.items()
    )


def plan_route_tags(
    *,
    route: RouteLiteral,
    surfaces: tuple[SurfacePlanRecord, ...],
    volumes: tuple[VolumePlanRecord, ...],
) -> tuple[TagPlanRecord, ...]:
    """Plan physical names before backend entity tags exist."""
    tags: list[TagPlanRecord] = []
    tags.extend(
        TagPlanRecord(
            physical_name=volume.owner_semantic_id,
            dimension=3,
            source_record_kind="volume",
            source_record_id=volume.volume_id,
            role="material_volume",
        )
        for volume in volumes
        if not volume.construction_only
    )
    tags.extend(
        TagPlanRecord(
            physical_name=_surface_physical_name(surface),
            dimension=2,
            source_record_kind="surface",
            source_record_id=surface.surface_id,
            role=surface.surface_role,
            solver_use=surface.solver_use,
        )
        for surface in surfaces
        if not surface.construction_only
    )
    return tuple(tags)


def _geometry_ref_from_metadata(metadata: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: metadata[key]
        for key in _GEOMETRY_REF_METADATA_KEYS
        if key in metadata
    }


def _interface_kinds(interface: InterfacePlanRecord) -> tuple[str, ...]:
    raw = interface.metadata.get("interface_kinds", (interface.kind,))
    if isinstance(raw, str):
        raw_values = (raw,)
    elif isinstance(raw, tuple | list | set):
        raw_values = raw
    else:
        raw_values = (interface.kind,)

    seen = {str(value) for value in raw_values}
    seen.add(interface.kind)
    return tuple(kind for kind in _INTERFACE_KIND_ORDER if kind in seen)


def _surface_physical_name(surface: SurfacePlanRecord) -> str:
    raw = surface.metadata.get("interface_kinds", ())
    if isinstance(raw, str):
        kinds = (raw,)
    elif isinstance(raw, tuple | list | set):
        kinds = tuple(str(kind) for kind in raw)
    else:
        kinds = ()
    if not kinds:
        return surface.surface_id
    return f"{'_'.join(kinds)}__{surface.surface_id}"


def _entity_geometry_ref(
    entity: SemanticEntitySpec,
    *,
    representation: str,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "from_semantic_id": entity.semantic_id,
        "geometry_kind": entity.geometry_kind,
        "part_role": entity.part_role,
        "representation": representation,
        "source_polygon_ids": entity.polygon_ids,
    }
    result.update(_geometry_ref_from_metadata(entity.geometry))
    result.update(_geometry_ref_from_metadata(entity.metadata))
    if "z_um" in entity.geometry and "plane" not in result:
        result["plane"] = {"axis": "z", "value_um": entity.geometry["z_um"]}
    return result


def _default_host_solution_id(build_input: GeometryBuildInput) -> str:
    for entity in build_input.entities:
        if _is_air_like_solution_entity(entity):
            return entity.semantic_id
    raise ValueError("GeometryBuildInput requires an air/vacuum host solution entity")

