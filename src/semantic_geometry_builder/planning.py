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

Volume construction contract:

All routes build topology surface-first. The planner must create or reuse every
boundary surface before it emits a backend-live volume. `AIR`, substrate, and
Route C material volumes are then assembled only from `SurfaceRefRecord`s with
`addSurfaceLoop()` followed by `addVolume()`. Domain bounds may be kept for
audit and for planning outer boundary faces, but they must not become direct
box/extrude fallback volumes.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from math import sqrt
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
    "z_um",
    "thickness_um",
)
_INTERFACE_KIND_ORDER = ("MM", "SS", "AA", "MS", "MA", "SA")
_INSET_EPS_UM = 1e-9


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
    surfaces, generated_surface_partitions = apply_inset_surface_partitions(
        build_input,
        route=route,
        surfaces=surfaces,
    )
    surface_partitions = (*surface_partitions, *generated_surface_partitions)
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
    intents = dict(build_input.metadata.get("interface_intents_2d", {}))
    records: list[InterfacePlanRecord] = []
    for index, intent in enumerate(intents.get("interfaces", ())):
        if not isinstance(intent, Mapping):
            raise ValueError("interfaces entries must be mappings")
        if not _intent_supports_route(intent, route):
            continue
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
        if not _intent_supports_route(intent, route):
            continue
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
        if not _intent_supports_route(intent, route):
            continue
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


def apply_inset_surface_partitions(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> tuple[tuple[SurfacePlanRecord, ...], tuple[SurfacePartitionRecord, ...]]:
    """Replace eligible parent interface surfaces with inset child surfaces.

    Planar surfaces use real 2D inward offsets. Sidewall surfaces use
    sidewall-specific strip partitioning along the shorter local axis: requested
    thresholds that are not strictly smaller than that axis length are skipped,
    and the remaining area becomes `CORE`. This avoids 500 nm / 1 um bands
    degenerating on a 200 nm metal-thickness sidewall while keeping every child
    surface backend-buildable.
    """
    breakpoints_um = _inset_breakpoints_for_route(build_input, route)
    if len(breakpoints_um) <= 1:
        return surfaces, ()

    child_surfaces: list[SurfacePlanRecord] = []
    partitions: list[SurfacePartitionRecord] = []
    for surface in surfaces:
        if (
            surface.interface_id is None
            or surface.parent_surface_id is not None
            or surface.construction_only
        ):
            child_surfaces.append(surface)
            continue

        inset_geometry_refs = _inset_geometry_refs(
            surface.geometry_ref,
            breakpoints_um,
        )
        if not inset_geometry_refs:
            raise ValueError(
                f"{surface.surface_id} cannot be partitioned into nondegenerate "
                "inset child surfaces"
            )

        parent_physical_name = _surface_physical_name(surface)
        for index, (label, band_min_um, band_max_um, geometry_ref) in enumerate(
            inset_geometry_refs
        ):
            child_surface_id = f"{surface.surface_id}__{label}__P{index:04d}"
            partition_id = f"PART__{surface.interface_id}__{label}__P{index:04d}"
            inset_band = {
                "label": label,
                "min_um": band_min_um,
                "max_um": band_max_um,
            }
            metadata = {
                **dict(surface.metadata),
                "inset_band": inset_band,
                "physical_name": f"{parent_physical_name}__{label}",
            }
            child_surfaces.append(
                SurfacePlanRecord(
                    surface_id=child_surface_id,
                    owner_semantic_id=surface.owner_semantic_id,
                    surface_role=f"{surface.surface_role}_inset",
                    geometry_ref={**geometry_ref, "inset_band": inset_band},
                    interface_id=surface.interface_id,
                    parent_surface_id=surface.surface_id,
                    partition_label=label,
                    normal_hint=surface.normal_hint,
                    valid_routes=surface.valid_routes,
                    solver_use=surface.solver_use,
                    construction_only=surface.construction_only,
                    metadata=metadata,
                )
            )
            partitions.append(
                SurfacePartitionRecord(
                    partition_id=partition_id,
                    parent_interface_id=surface.interface_id,
                    child_surface_id=child_surface_id,
                    label=label,
                    band_min_um=band_min_um,
                    band_max_um=band_max_um,
                    parameterization=(
                        "planar_uv"
                        if "quad_points" not in geometry_ref
                        else "occ_native_parametric"
                    ),
                    valid_routes=(route,),
                    metadata={
                        "inset_band": inset_band,
                        "parent_surface_id": surface.surface_id,
                    },
                )
            )
    return tuple(child_surfaces), tuple(partitions)


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
                expected_surface_ids=_cutout_shell_surface_ids(route, entity),
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
    records.extend(_plan_substrate_air_surfaces(build_input, route=route))
    for interface in interfaces:
        geometry_ref = {
            "from_interface_id": interface.interface_id,
            **_geometry_ref_from_metadata(interface.metadata),
        }
        interface_kinds = _interface_surface_kinds(
            build_input,
            route=route,
            interface=interface,
        )
        owner_semantic_ids = _interface_surface_owner_ids(
            build_input,
            route=route,
            interface=interface,
        )
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
                            "interface_kinds": interface_kinds,
                            "owner_semantic_ids": owner_semantic_ids,
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
                    "interface_kinds": interface_kinds,
                    "owner_semantic_ids": owner_semantic_ids,
                },
            )
        )

    construction_body_by_surface_id = {
        surface_id: body
        for body in construction_bodies
        for surface_id in body.expected_surface_ids
    }
    default_host = _default_host_solution_id(build_input)
    default_substrate = _default_substrate_solution_id(build_input)
    for entity in build_input.entities:
        if _is_solution_entity(entity):
            continue
        representation = entity.route_representations.get(route)
        if representation in {"cutout_boundary_shell", "material_volume"}:
            for shell_part, interface_kind, adjacent_id in (
                ("top", "MA", default_host),
                ("bottom", "MS", default_substrate),
            ):
                surface_id = _conductor_boundary_surface_id(
                    route,
                    entity,
                    representation,
                    shell_part,
                )
                body = construction_body_by_surface_id.get(surface_id)
                records.append(
                    SurfacePlanRecord(
                        surface_id=surface_id,
                        owner_semantic_id=entity.semantic_id,
                        surface_role=(
                            "cutout_boundary_shell"
                            if representation == "cutout_boundary_shell"
                            else "material_interface"
                        ),
                        geometry_ref={
                            **_entity_geometry_ref(
                                entity,
                                representation=representation,
                            ),
                            "shell_part": shell_part,
                            "construction_body_id": (
                                body.construction_body_id
                                if body is not None
                                else None
                            ),
                        },
                        interface_id=(
                            f"{interface_kind}__{entity.semantic_id}__"
                            f"{adjacent_id}__{shell_part.upper()}"
                        ),
                        valid_routes=(route,),
                        solver_use="solver_active",
                        metadata={
                            "interface_kinds": (interface_kind,),
                            "owner_semantic_ids": (
                                entity.semantic_id,
                                adjacent_id,
                            ),
                            "exposed_surface_role": shell_part,
                        },
                    )
                )
            for edge_index, geometry_ref in enumerate(
                _sidewall_geometry_refs(
                    _entity_geometry_ref(entity, representation=representation)
                )
            ):
                shell_part = f"sidewall_{edge_index:04d}"
                surface_id = _conductor_boundary_surface_id(
                    route,
                    entity,
                    representation,
                    shell_part,
                )
                body = construction_body_by_surface_id.get(surface_id)
                records.append(
                    SurfacePlanRecord(
                        surface_id=surface_id,
                        owner_semantic_id=entity.semantic_id,
                        surface_role=(
                            "cutout_boundary_shell"
                            if representation == "cutout_boundary_shell"
                            else "material_interface"
                        ),
                        geometry_ref={
                            **geometry_ref,
                            "from_semantic_id": entity.semantic_id,
                            "geometry_kind": entity.geometry_kind,
                            "part_role": entity.part_role,
                            "representation": representation,
                            "source_polygon_ids": entity.polygon_ids,
                            "shell_part": shell_part,
                            "construction_body_id": (
                                body.construction_body_id
                                if body is not None
                                else None
                            ),
                        },
                        interface_id=(
                            f"MA__{entity.semantic_id}__{default_host}__"
                            f"{shell_part.upper()}"
                        ),
                        valid_routes=(route,),
                        solver_use="solver_active",
                        metadata={
                            "interface_kinds": ("MA",),
                            "owner_semantic_ids": (
                                entity.semantic_id,
                                default_host,
                            ),
                            "exposed_surface_role": shell_part,
                        },
                    )
                )
    return tuple(records)


def plan_route_volumes(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> tuple[VolumePlanRecord, ...]:
    """Plan volumes only after all boundary surfaces have stable ids.

    The same rule applies to Route A/B/C: solution domains and retained Route C
    conductors are closed by planned surfaces, then lowered through
    `addSurfaceLoop()` and `addVolume()`. This function must not create a
    volume from `domain_bounds_um`, `outer_loop`, or `thickness_um`; those values
    are only audit/planning metadata once surface ids exist.
    """
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
                    metadata={
                        "representation": "solution_volume",
                        "geometry_ref": dict(entity.geometry),
                    },
                )
            )
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
                    metadata={
                        "representation": representation,
                        "geometry_ref": _entity_geometry_ref(
                            entity,
                            representation=representation,
                        ),
                    },
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
    """Plan physical names before backend entity tags exist.

    Domain boundary surfaces stay in the construction plan because they close
    AIR/substrate volumes, but they are not exported as surface physical groups
    in v1. Surface physical groups are reserved for semantic interfaces such as
    SA/MS/MA/MM/SS/AA.
    """
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
        if not surface.construction_only and surface.interface_id is not None
    )
    return tuple(tags)


def _geometry_ref_from_metadata(metadata: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: metadata[key]
        for key in _GEOMETRY_REF_METADATA_KEYS
        if key in metadata
    }


def _intent_supports_route(intent: Mapping[str, Any], route: RouteLiteral) -> bool:
    valid_routes = intent.get("valid_routes")
    if valid_routes is None:
        return True
    if isinstance(valid_routes, str):
        return route == valid_routes
    return route in {str(value) for value in valid_routes}


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


def _interface_surface_kinds(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> tuple[str, ...]:
    del build_input
    kinds = set(_interface_kinds(interface))
    if route == "A" and interface.metadata.get(
        "recognition_rule"
    ) == "route_a_surface_sheet_polygon":
        kinds.update(("MS", "MA", "SA"))
    return tuple(kind for kind in _INTERFACE_KIND_ORDER if kind in kinds)


def _interface_surface_owner_ids(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> tuple[str, ...]:
    owner_ids = list(interface.owner_semantic_ids)
    if route == "A" and interface.metadata.get(
        "recognition_rule"
    ) == "route_a_surface_sheet_polygon":
        substrate_id = _default_substrate_solution_id(build_input)
        if substrate_id not in owner_ids:
            owner_ids.append(substrate_id)
    return tuple(owner_ids)


def _surface_physical_name(surface: SurfacePlanRecord) -> str:
    override = surface.metadata.get("physical_name")
    if isinstance(override, str) and override:
        return override
    raw = surface.metadata.get("interface_kinds", ())
    if isinstance(raw, str):
        kinds = (raw,)
    elif isinstance(raw, tuple | list | set):
        kinds = tuple(str(kind) for kind in raw)
    else:
        kinds = ()
    kind_prefix = "_".join(kinds)
    exposed_role = surface.metadata.get("exposed_surface_role")
    boundary_role = surface.metadata.get("boundary_role")
    if surface.interface_id is not None:
        parts = surface.interface_id.split("__")
        if isinstance(exposed_role, str) and exposed_role.startswith("sidewall_"):
            parts[-1] = "SIDEWALL"
        if parts and parts[0] in _INTERFACE_KIND_ORDER:
            return "__".join((kind_prefix or parts[0], *parts[1:]))
        return surface.interface_id
    if kind_prefix:
        owner_ids = surface.metadata.get(
            "owner_semantic_ids",
            (surface.owner_semantic_id,),
        )
        if isinstance(owner_ids, str):
            owner_ids = (owner_ids,)
        suffix = exposed_role
        parts = [kind_prefix, *(str(owner_id) for owner_id in owner_ids)]
        if suffix:
            parts.append(str(suffix).upper())
        return "__".join(parts)
    if boundary_role == "sidewall":
        return surface.surface_id.removeprefix("SURF__").rsplit("__", 1)[0]
    return surface.surface_id.removeprefix("SURF__")


def _plan_substrate_air_surfaces(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
) -> tuple[SurfacePlanRecord, ...]:
    substrate = _entity_by_id(build_input, _default_substrate_solution_id(build_input))
    air_id = _default_host_solution_id(build_input)
    substrate_bounds = substrate.geometry.get("domain_bounds_um")
    if not isinstance(substrate_bounds, Mapping):
        return ()

    conductor_entities = [
        entity
        for entity in build_input.entities
        if not _is_solution_entity(entity)
        and entity.route_representations.get(route) is not None
        and "outer_loop" in entity.geometry
    ]
    derived_ground_entities = [
        entity
        for entity in conductor_entities
        if entity.geometry.get("geometry_source") == "die_face_minus_ground_mask"
    ]
    if derived_ground_entities:
        base_loops = tuple(
            loop
            for entity in derived_ground_entities
            for loop in entity.geometry.get("hole_loops", ())
        )
        hole_entities = [
            entity
            for entity in conductor_entities
            if entity not in derived_ground_entities
        ]
    else:
        base_loops = (_domain_bounds_loop(substrate_bounds),)
        hole_entities = conductor_entities

    z_um = float(substrate.geometry.get("z_max_um", 0.0))
    records: list[SurfacePlanRecord] = list(
        _plan_solution_domain_boundary_surfaces(build_input, route=route)
    )
    for index, outer_loop in enumerate(base_loops):
        hole_loops = tuple(
            entity.geometry["outer_loop"]
            for entity in hole_entities
            if _loop_inside_loop(entity.geometry["outer_loop"], outer_loop)
        )
        interface_id = f"SA__{substrate.semantic_id}__{air_id}__{index:04d}"
        records.append(
            SurfacePlanRecord(
                surface_id=f"SURF__{interface_id}",
                owner_semantic_id=substrate.semantic_id,
                surface_role="solution_interface",
                geometry_ref={
                    "plane": {"axis": "z", "value_um": z_um},
                    "outer_loop": outer_loop,
                    "hole_loops": hole_loops,
                },
                interface_id=interface_id,
                valid_routes=(route,),
                solver_use="solver_active",
                metadata={
                    "interface_kinds": ("SA",),
                    "owner_semantic_ids": (substrate.semantic_id, air_id),
                },
            )
        )
    return tuple(records)


def _plan_solution_domain_boundary_surfaces(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
) -> tuple[SurfacePlanRecord, ...]:
    records: list[SurfacePlanRecord] = []
    for entity in build_input.entities:
        if not _is_solution_entity(entity):
            continue
        bounds = entity.geometry.get("domain_bounds_um")
        if not isinstance(bounds, Mapping):
            continue
        z_min_um = float(entity.geometry["z_min_um"])
        z_max_um = float(entity.geometry["z_max_um"])
        loop = _domain_bounds_loop(bounds)
        boundary_parts = (
            ("top", z_max_um)
            if _is_air_like_solution_entity(entity)
            else ("bottom", z_min_um)
        )
        records.append(
            SurfacePlanRecord(
                surface_id=(
                    f"SURF__BOUNDARY__{entity.semantic_id}__"
                    f"{boundary_parts[0].upper()}"
                ),
                owner_semantic_id=entity.semantic_id,
                surface_role="domain_boundary",
                geometry_ref={
                    "plane": {"axis": "z", "value_um": boundary_parts[1]},
                    "outer_loop": loop,
                    "hole_loops": (),
                },
                valid_routes=(route,),
                metadata={
                    "owner_semantic_ids": (entity.semantic_id,),
                    "boundary_role": boundary_parts[0],
                },
            )
        )
        for edge_index, geometry_ref in enumerate(
            _sidewall_geometry_refs(
                {
                    "outer_loop": loop,
                    "hole_loops": (),
                    "z_min_um": z_min_um,
                    "thickness_um": z_max_um - z_min_um,
                }
            )
        ):
            records.append(
                SurfacePlanRecord(
                    surface_id=(
                        f"SURF__BOUNDARY__{entity.semantic_id}__"
                        f"SIDEWALL__{edge_index:04d}"
                    ),
                    owner_semantic_id=entity.semantic_id,
                    surface_role="domain_boundary",
                    geometry_ref=geometry_ref,
                    valid_routes=(route,),
                    metadata={
                        "owner_semantic_ids": (entity.semantic_id,),
                        "boundary_role": "sidewall",
                    },
                )
            )
    return tuple(records)


def _cutout_shell_surface_ids(
    route: RouteLiteral,
    entity: SemanticEntitySpec,
) -> tuple[str, ...]:
    top_bottom_ids = tuple(
        _conductor_boundary_surface_id(
            route,
            entity,
            "cutout_boundary_shell",
            shell_part,
        )
        for shell_part in ("top", "bottom")
    )
    sidewall_ids = tuple(
        _conductor_boundary_surface_id(
            route,
            entity,
            "cutout_boundary_shell",
            f"sidewall_{edge_index:04d}",
        )
        for edge_index, _ in enumerate(
            _sidewall_geometry_refs(
                _entity_geometry_ref(
                    entity,
                    representation="cutout_boundary_shell",
                )
            )
        )
    )
    return (*top_bottom_ids, *sidewall_ids)


def _conductor_boundary_surface_id(
    route: RouteLiteral,
    entity: SemanticEntitySpec,
    representation: str,
    shell_part: str,
) -> str:
    surface_kind = "SHELL" if representation == "cutout_boundary_shell" else "MAT"
    return f"SURF__{route}__{surface_kind}__{entity.semantic_id}__{shell_part.upper()}"


def _entity_by_id(
    build_input: GeometryBuildInput,
    semantic_id: str,
) -> SemanticEntitySpec:
    for entity in build_input.entities:
        if entity.semantic_id == semantic_id:
            return entity
    raise ValueError(f"unknown semantic entity: {semantic_id}")


def _domain_bounds_loop(bounds: Mapping[str, Any]) -> tuple[tuple[float, float], ...]:
    return (
        (float(bounds["x_min_um"]), float(bounds["y_min_um"])),
        (float(bounds["x_max_um"]), float(bounds["y_min_um"])),
        (float(bounds["x_max_um"]), float(bounds["y_max_um"])),
        (float(bounds["x_min_um"]), float(bounds["y_max_um"])),
    )


def _loop_inside_loop(loop: Any, container: Any) -> bool:
    x, y = _loop_centroid(loop)
    return _point_in_loop((x, y), container)


def _loop_centroid(loop: Any) -> tuple[float, float]:
    points = tuple((float(point[0]), float(point[1])) for point in loop)
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _point_in_loop(point: tuple[float, float], loop: Any) -> bool:
    x, y = point
    points = tuple((float(item[0]), float(item[1])) for item in loop)
    inside = False
    previous = len(points) - 1
    for index, (xi, yi) in enumerate(points):
        xj, yj = points[previous]
        if (yi > y) != (yj > y):
            x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_intersect:
                inside = not inside
        previous = index
    return inside


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


def _sidewall_geometry_refs(
    geometry_ref: Mapping[str, Any],
) -> tuple[dict[str, Any], ...]:
    z_min_um = float(geometry_ref.get("z_min_um", geometry_ref.get("z_um", 0.0)))
    thickness_um = float(geometry_ref.get("thickness_um", 0.0))
    if thickness_um <= 0:
        return ()
    z_max_um = z_min_um + thickness_um
    refs: list[dict[str, Any]] = []
    for ring_role, ring in (
        ("outer", geometry_ref["outer_loop"]),
        *(
            (f"hole_{index:04d}", hole_loop)
            for index, hole_loop in enumerate(geometry_ref.get("hole_loops", ()))
        ),
    ):
        for edge_index, (start, end) in enumerate(_ring_edges(_clean_loop(ring))):
            refs.append(
                {
                    "quad_points": (
                        (start[0], start[1], z_min_um),
                        (end[0], end[1], z_min_um),
                        (end[0], end[1], z_max_um),
                        (start[0], start[1], z_max_um),
                    ),
                    "sidewall_ring_role": ring_role,
                    "sidewall_edge_index": edge_index,
                }
            )
    return tuple(refs)


def _clean_loop(loop: Any) -> tuple[tuple[float, float], ...]:
    points = tuple((float(point[0]), float(point[1])) for point in loop)
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("loop requires at least 3 unique points")
    return points


def _ring_edges(
    ring: tuple[tuple[float, float], ...],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    return tuple(
        (ring[index], ring[(index + 1) % len(ring)])
        for index in range(len(ring))
    )


def _inset_breakpoints_for_route(
    build_input: GeometryBuildInput,
    route: RouteLiteral,
) -> tuple[float, ...]:
    raw_routes = build_input.metadata.get("inset_routes")
    if raw_routes is not None:
        if isinstance(raw_routes, str):
            valid_routes = {raw_routes}
        elif isinstance(raw_routes, Sequence):
            valid_routes = {str(value) for value in raw_routes}
        else:
            raise TypeError("inset_routes metadata must be a string or sequence")
        if route not in valid_routes:
            return ()

    raw_breakpoints = build_input.metadata.get("inset_breakpoints_um", ())
    if raw_breakpoints in (None, ()):
        return ()
    if isinstance(raw_breakpoints, str) or not isinstance(raw_breakpoints, Sequence):
        raise TypeError("inset_breakpoints_um metadata must be a numeric sequence")
    breakpoints = tuple(float(value) for value in raw_breakpoints)
    if not breakpoints:
        return ()
    if breakpoints[0] != 0.0:
        raise ValueError("inset_breakpoints_um must start at 0.0")
    if tuple(sorted(breakpoints)) != breakpoints:
        raise ValueError("inset_breakpoints_um must be sorted")
    if len(set(breakpoints)) != len(breakpoints):
        raise ValueError("inset_breakpoints_um must not contain duplicates")
    if any(value < 0.0 for value in breakpoints):
        raise ValueError("inset_breakpoints_um must be non-negative")
    return breakpoints


def _inset_geometry_refs(
    geometry_ref: Mapping[str, Any],
    breakpoints_um: tuple[float, ...],
) -> tuple[tuple[str, float | None, float | None, dict[str, Any]], ...]:
    if "quad_points" in geometry_ref:
        return _sidewall_inset_geometry_refs(geometry_ref, breakpoints_um)
    if "outer_loop" in geometry_ref:
        return _planar_inset_geometry_refs(geometry_ref, breakpoints_um)
    raise ValueError("inset surfaces require outer_loop or quad_points geometry")


def _planar_inset_geometry_refs(
    geometry_ref: Mapping[str, Any],
    breakpoints_um: tuple[float, ...],
) -> tuple[tuple[str, float | None, float | None, dict[str, Any]], ...]:
    import gdstk

    refs: list[tuple[str, float | None, float | None, dict[str, Any]]] = []
    outer_loop = _clean_loop(geometry_ref["outer_loop"])
    hole_loops = tuple(
        _clean_loop(hole_loop)
        for hole_loop in geometry_ref.get("hole_loops", ())
    )

    for band_min_um, band_max_um in zip(
        breakpoints_um,
        breakpoints_um[1:],
        strict=False,
    ):
        label = _inset_band_label(band_min_um, band_max_um)
        refs.extend(
            (
                label,
                band_min_um,
                band_max_um,
                geometry_ref_for_band,
            )
            for geometry_ref_for_band in _planar_inset_band_refs(
                gdstk,
                geometry_ref,
                outer_loop=outer_loop,
                hole_loops=hole_loops,
                band_min_um=band_min_um,
                band_max_um=band_max_um,
            )
        )

    core_min_um = breakpoints_um[-1]
    refs.extend(
        (
            _core_label(core_min_um),
            core_min_um,
            None,
            geometry_ref_for_core,
        )
        for geometry_ref_for_core in _planar_inset_core_refs(
            gdstk,
            geometry_ref,
            outer_loop=outer_loop,
            hole_loops=hole_loops,
            core_min_um=core_min_um,
        )
    )
    return tuple(refs)


def _planar_inset_band_refs(
    gdstk: Any,
    geometry_ref: Mapping[str, Any],
    *,
    outer_loop: tuple[tuple[float, float], ...],
    hole_loops: tuple[tuple[tuple[float, float], ...], ...],
    band_min_um: float,
    band_max_um: float,
) -> tuple[dict[str, Any], ...]:
    refs: list[dict[str, Any]] = []

    outer_band_loops = _offset_loop_candidates(gdstk, outer_loop, -band_min_um)
    outer_band_holes = _offset_loop_candidates(gdstk, outer_loop, -band_max_um)
    refs.extend(
        _geometry_ref_with_loops(
            geometry_ref,
            outer,
            tuple(hole for hole in outer_band_holes if _loop_inside_loop(hole, outer)),
        )
        for outer in outer_band_loops
    )

    for hole_loop in hole_loops:
        hole_band_loops = _offset_loop_candidates(gdstk, hole_loop, band_max_um)
        hole_band_holes = _offset_loop_candidates(gdstk, hole_loop, band_min_um)
        refs.extend(
            _geometry_ref_with_loops(
                geometry_ref,
                outer,
                tuple(
                    hole
                    for hole in hole_band_holes
                    if _loop_inside_loop(hole, outer)
                ),
            )
            for outer in hole_band_loops
        )
    return tuple(ref for ref in refs if ref["outer_loop"])


def _planar_inset_core_refs(
    gdstk: Any,
    geometry_ref: Mapping[str, Any],
    *,
    outer_loop: tuple[tuple[float, float], ...],
    hole_loops: tuple[tuple[tuple[float, float], ...], ...],
    core_min_um: float,
) -> tuple[dict[str, Any], ...]:
    core_outer_loops = _offset_loop_candidates(gdstk, outer_loop, -core_min_um)
    expanded_holes = tuple(
        expanded_hole
        for hole_loop in hole_loops
        for expanded_hole in _offset_loop_candidates(gdstk, hole_loop, core_min_um)
    )
    return tuple(
        _geometry_ref_with_loops(
            geometry_ref,
            outer,
            tuple(hole for hole in expanded_holes if _loop_inside_loop(hole, outer)),
        )
        for outer in core_outer_loops
    )


def _sidewall_inset_geometry_refs(
    geometry_ref: Mapping[str, Any],
    breakpoints_um: tuple[float, ...],
) -> tuple[tuple[str, float | None, float | None, dict[str, Any]], ...]:
    points = tuple(
        (float(point[0]), float(point[1]), float(point[2]))
        for point in geometry_ref["quad_points"]
    )
    if len(points) != 4:
        raise ValueError("quad_points requires exactly 4 points")
    width_um = _distance_3d(points[0], points[1])
    height_um = _distance_3d(points[0], points[3])
    if width_um <= _INSET_EPS_UM or height_um <= _INSET_EPS_UM:
        return ()

    limiting_um = min(width_um, height_um)
    valid_breakpoints = tuple(
        value
        for value in breakpoints_um
        if value == 0.0 or value < limiting_um - _INSET_EPS_UM
    )
    if not valid_breakpoints or valid_breakpoints[0] != 0.0:
        return ()

    axis = "u" if width_um <= height_um else "v"
    refs: list[tuple[str, float | None, float | None, dict[str, Any]]] = []
    for band_min_um, band_max_um in zip(
        valid_breakpoints,
        valid_breakpoints[1:],
        strict=False,
    ):
        quad_points = _sidewall_strip_points(
            points,
            width_um=width_um,
            height_um=height_um,
            axis=axis,
            start_um=band_min_um,
            end_um=band_max_um,
        )
        if quad_points is None:
            continue
        refs.append(
            (
                _inset_band_label(band_min_um, band_max_um),
                band_min_um,
                band_max_um,
                {
                    **dict(geometry_ref),
                    "quad_points": quad_points,
                    "sidewall_inset_rule": "short_axis_strips",
                },
            )
        )

    core_min_um = valid_breakpoints[-1]
    core_points = _sidewall_strip_points(
        points,
        width_um=width_um,
        height_um=height_um,
        axis=axis,
        start_um=core_min_um,
        end_um=limiting_um,
    )
    if core_points is not None:
        refs.append(
            (
                _core_label(core_min_um),
                core_min_um,
                None,
                {
                    **dict(geometry_ref),
                    "quad_points": core_points,
                    "sidewall_inset_rule": "short_axis_strips",
                },
            )
        )
    return tuple(refs)


def _sidewall_strip_points(
    points: tuple[tuple[float, float, float], ...],
    *,
    width_um: float,
    height_um: float,
    axis: str,
    start_um: float,
    end_um: float,
) -> tuple[tuple[float, float, float], ...] | None:
    if end_um - start_um <= _INSET_EPS_UM:
        return None
    if axis == "u":
        return (
            _quad_uv_point(points, width_um, height_um, start_um, 0.0),
            _quad_uv_point(points, width_um, height_um, end_um, 0.0),
            _quad_uv_point(points, width_um, height_um, end_um, height_um),
            _quad_uv_point(points, width_um, height_um, start_um, height_um),
        )
    return (
        _quad_uv_point(points, width_um, height_um, 0.0, start_um),
        _quad_uv_point(points, width_um, height_um, width_um, start_um),
        _quad_uv_point(points, width_um, height_um, width_um, end_um),
        _quad_uv_point(points, width_um, height_um, 0.0, end_um),
    )


def _quad_uv_point(
    points: tuple[tuple[float, float, float], ...],
    width_um: float,
    height_um: float,
    u_um: float,
    v_um: float,
) -> tuple[float, float, float]:
    origin = points[0]
    u_vec = _unit_vector(points[0], points[1], width_um)
    v_vec = _unit_vector(points[0], points[3], height_um)
    return (
        origin[0] + u_vec[0] * u_um + v_vec[0] * v_um,
        origin[1] + u_vec[1] * u_um + v_vec[1] * v_um,
        origin[2] + u_vec[2] * u_um + v_vec[2] * v_um,
    )


def _unit_vector(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    length: float,
) -> tuple[float, float, float]:
    return (
        (end[0] - start[0]) / length,
        (end[1] - start[1]) / length,
        (end[2] - start[2]) / length,
    )


def _distance_3d(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float:
    return sqrt(
        (end[0] - start[0]) ** 2
        + (end[1] - start[1]) ** 2
        + (end[2] - start[2]) ** 2
    )


def _gdstk_surface_region(geometry_ref: Mapping[str, Any]) -> tuple[Any, ...]:
    import gdstk

    outer = gdstk.Polygon(_clean_loop(geometry_ref["outer_loop"]))
    holes = tuple(
        gdstk.Polygon(_clean_loop(hole_loop))
        for hole_loop in geometry_ref.get("hole_loops", ())
    )
    if not holes:
        return (outer,)
    return _boolean_gdstk_region(gdstk, (outer,), holes, "not")


def _offset_gdstk_region(
    gdstk: Any,
    region: tuple[Any, ...],
    offset_um: float,
) -> tuple[Any, ...]:
    if offset_um <= _INSET_EPS_UM:
        return _filter_gdstk_polygons(region)
    return _filter_gdstk_polygons(
        gdstk.offset(
            region,
            -offset_um,
            join="miter",
            tolerance=1e-3,
            precision=1e-9,
        )
    )


def _boolean_gdstk_region(
    gdstk: Any,
    left: Sequence[Any],
    right: Sequence[Any],
    operation: str,
) -> tuple[Any, ...]:
    if not left:
        return ()
    if operation == "not" and not right:
        return _filter_gdstk_polygons(left)
    result = gdstk.boolean(
        left,
        right,
        operation,
        precision=1e-9,
    )
    return _filter_gdstk_polygons(result or ())


def _offset_loop_candidates(
    gdstk: Any,
    loop: tuple[tuple[float, float], ...],
    offset_um: float,
) -> tuple[tuple[tuple[float, float], ...], ...]:
    if abs(offset_um) <= _INSET_EPS_UM:
        return (loop,)
    polygons = _filter_gdstk_polygons(
        gdstk.offset(
            (gdstk.Polygon(loop),),
            offset_um,
            join="miter",
            tolerance=1e-3,
            precision=1e-9,
        )
    )
    return tuple(_clean_loop(polygon.points) for polygon in polygons)


def _geometry_ref_with_loops(
    parent_geometry_ref: Mapping[str, Any],
    outer_loop: tuple[tuple[float, float], ...],
    hole_loops: tuple[tuple[tuple[float, float], ...], ...],
) -> dict[str, Any]:
    geometry_ref = dict(parent_geometry_ref)
    geometry_ref["outer_loop"] = outer_loop
    geometry_ref["hole_loops"] = hole_loops
    return geometry_ref


def _filter_gdstk_polygons(polygons: Sequence[Any]) -> tuple[Any, ...]:
    return tuple(
        polygon
        for polygon in polygons
        if abs(_polygon_area(polygon.points)) > _INSET_EPS_UM
    )


def _geometry_ref_from_gdstk_polygon(
    parent_geometry_ref: Mapping[str, Any],
    polygon: Any,
) -> dict[str, Any]:
    geometry_ref = dict(parent_geometry_ref)
    geometry_ref["outer_loop"] = _clean_loop(polygon.points)
    geometry_ref["hole_loops"] = ()
    return geometry_ref


def _polygon_area(points: Sequence[Sequence[float]]) -> float:
    clean_points = tuple((float(point[0]), float(point[1])) for point in points)
    if len(clean_points) < 3:
        return 0.0
    return 0.5 * sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in _ring_edges(clean_points)
    )


def _inset_band_label(band_min_um: float, band_max_um: float) -> str:
    return f"RING_{_distance_label(band_min_um)}_{_distance_label(band_max_um)}"


def _core_label(band_min_um: float) -> str:
    return f"CORE_AFTER_{_distance_label(band_min_um)}"


def _distance_label(value_um: float) -> str:
    value_nm = value_um * 1000.0
    if abs(value_nm - round(value_nm)) < 1e-6:
        value_nm_int = int(round(value_nm))
        if value_nm_int >= 1000 and value_nm_int % 1000 == 0:
            return f"{value_nm_int // 1000}UM"
        return f"{value_nm_int}NM"
    return f"{value_nm:g}NM"


def _default_host_solution_id(build_input: GeometryBuildInput) -> str:
    for entity in build_input.entities:
        if _is_air_like_solution_entity(entity):
            return entity.semantic_id
    raise ValueError("GeometryBuildInput requires an air/vacuum host solution entity")


def _default_substrate_solution_id(build_input: GeometryBuildInput) -> str:
    for entity in build_input.entities:
        if not _is_solution_entity(entity):
            continue
        tokens = {
            entity.semantic_id.lower(),
            entity.role.lower(),
            entity.material_id.lower(),
            entity.geometry_kind.lower(),
        }
        if any("substrate" in token for token in tokens):
            return entity.semantic_id
    raise ValueError("GeometryBuildInput requires a substrate solution entity")
