"""Route-first topology planning for semantic geometry v1.

This module owns semantic planning only. It recognizes interface intent, expands
inset children, canonicalizes points/curves/surfaces, plans Route A/B
construction cuts, Route C retained volumes, and physical tag intent before any
backend entity tag exists.

The intended compiler stages are:

1. normalize GDS/stack geometry onto one canonical grid, fix ring orientation,
   split multipolygons, and reject self-intersections or slivers;
2. build a 2D planar arrangement by splitting intersections, shared overlaps,
   and T-junctions into atomic points, edges, and cells;
3. sweep those cells through stack z-events to resolve material/domain
   occupancy for each 3D cell;
4. derive `InterfacePlanRecord`s from horizontal and vertical adjacency;
5. emit canonical point, curve, surface, volume, and tag plans.

`interface_intents_2d` can seed or request interface behavior, but it must not
be the only source of solver-relevant interfaces. Actual interface plans come
from topology adjacency.

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
  When adjacent interface parents are coplanar, such as `SA` and `MS` on one
  substrate face, inset is a property of the shared plane arrangement rather
  than of either parent alone. The planner must generate one coplanar inset
  family, then emit child surfaces for each interface label from that shared
  topology graph.

Route outputs:

- Route A: interface-owned `surface_sheet` conductors for face metal and
  airbridge decks, plus `cutout_boundary_shell` PEC surfaces for bumps/posts.
- Route B: `cutout_boundary_shell` PEC surfaces from construction bodies.
- Route C: retained `material_volume` records assembled from planned shared
  surfaces.

High-count local conductors such as indium bumps and airbridges keep
per-instance topology records, but physical groups are planned at the semantic
family/context level. Instance ids like `_0000` may remain in `SurfacePlanRecord`
or `VolumePlanRecord` source ids; they must not leak into final physical names
when `semantic_group_id` identifies the die-to-die conductor family.

Volume construction contract:

All routes build topology surface-first. The planner must create or reuse every
boundary surface before it emits a backend-live volume. `AIR`, substrate, and
Route C material volumes are then assembled only from `SurfaceRefRecord`s with
`addSurfaceLoop()` followed by `addVolume()`. Domain bounds may be kept for
audit and for planning outer boundary faces, but they must not become direct
box/extrude fallback volumes.

Canonical topology contract:

The compiler, not the Gmsh backend, owns conformal topology identity. Planned
surface candidates must be converted into canonical `PointPlanRecord`s,
`CurvePlanRecord`s, and `SurfaceLoopRecord`s before backend lowering. That
registry is where shared vertices, shared edges, inset child coverage,
duplicate surfaces, and volume shell closure become reviewable metadata.
Backend point/line caches may still exist as a lowering optimization, but they
must not be the proof that two surfaces share topology.

Hard invariants for this registry:

- no planned curve may contain another planned point in its interior;
- parent interface surfaces must be replaced by non-overlapping child
  ring/core patches, never kept live beside those children;
- coplanar inset children that share points, curves, or boundary-volume
  ownership must come from one joint planar arrangement, not independent
  per-surface offset operations;
- every MA/MS/MM/SA/SS/AA surface must trace back to `InterfacePlanRecord`;
- live surfaces on the same plane must not have duplicate or overlapping area;
- volume shells must be checkable from planned curve incidence before OCC runs.
- internal shared surfaces must be referenced by both adjacent volumes; exterior
  surfaces must be referenced by exactly one live volume.
"""

from __future__ import annotations

import time
from bisect import bisect_left, bisect_right
from collections.abc import Mapping, Sequence
from dataclasses import replace
from math import sqrt
from typing import Any

from semantic_geometry_builder.models import (
    HIGH_COUNT_LOCAL_CONDUCTOR_PART_ROLES,
    ConstructionBodyPlanRecord,
    ConstructionPlanRecord,
    CoplanarInsetFamilyRecord,
    CurvePlanRecord,
    CurveRefRecord,
    CutHostOperationRecord,
    GeometryBuildInput,
    InterfacePlanRecord,
    MeshSizeHintRecord,
    PointPlanRecord,
    RouteLiteral,
    SemanticEntitySpec,
    SurfaceLoopRecord,
    SurfaceOrientationLiteral,
    SurfacePartitionRecord,
    SurfacePlanRecord,
    SurfaceRefRecord,
    TagPlanRecord,
    VolumePlanRecord,
)
from semantic_geometry_builder.validation import (
    _is_air_like_solution_entity,
    _is_solution_entity,
    validate_curve_plan_coverage,
    validate_inset_mesh_contract,
    validate_interface_surface_source_of_truth,
    validate_no_surface_overlap,
    validate_route_operation_coverage,
    validate_route_volume_surface_refs,
    validate_selected_route,
    validate_surface_deduplication,
    validate_surface_partition_coverage,
    validate_surface_sheet_interface_coverage,
    validate_surface_use_counts,
    validate_tag_plan_coverage,
    validate_volume_surface_closure,
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
_SOLUTION_ENTITIES_BY_INPUT_ID: dict[
    int,
    tuple[GeometryBuildInput, tuple[SemanticEntitySpec, ...]],
] = {}


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
    timings: list[dict[str, Any]] = []
    _timed(
        timings,
        "validate_selected_route",
        lambda: validate_selected_route(build_input, route),
    )
    interfaces = _timed(
        timings,
        "recognize_route_interfaces",
        lambda: recognize_route_interfaces(build_input, route=route),
    )
    interfaces = _timed(
        timings,
        "plan_conductor_contact_patches",
        lambda: plan_conductor_contact_patches(
            build_input,
            route=route,
            interfaces=interfaces,
        ),
    )
    surface_partitions = _timed(
        timings,
        "plan_surface_partitions",
        lambda: plan_surface_partitions(
            build_input,
            route=route,
            interfaces=interfaces,
        ),
    )
    construction_bodies = _timed(
        timings,
        "plan_route_construction_bodies",
        lambda: plan_route_construction_bodies(
            build_input,
            route=route,
            interfaces=interfaces,
        ),
    )
    surfaces = _timed(
        timings,
        "plan_route_surfaces",
        lambda: plan_route_surfaces(
            build_input,
            route=route,
            interfaces=interfaces,
            surface_partitions=surface_partitions,
            construction_bodies=construction_bodies,
        ),
    )
    surfaces, generated_surface_partitions = _timed(
        timings,
        "apply_inset_surface_partitions",
        lambda: apply_inset_surface_partitions(
            build_input,
            route=route,
            surfaces=surfaces,
        ),
    )
    surface_partitions = (*surface_partitions, *generated_surface_partitions)
    coplanar_inset_families = _timed(
        timings,
        "plan_coplanar_inset_families",
        lambda: plan_coplanar_inset_families(
            surfaces=surfaces,
            surface_partitions=surface_partitions,
            breakpoints_um=_inset_breakpoints_for_route(build_input, route),
        ),
    )
    _timed(
        timings,
        "validate_coplanar_inset_family_contract",
        lambda: validate_coplanar_inset_family_contract(
            surfaces=surfaces,
            coplanar_inset_families=coplanar_inset_families,
        ),
    )
    mesh_size_hints = _timed(
        timings,
        "plan_mesh_size_hints",
        lambda: plan_mesh_size_hints(surface_partitions=surface_partitions),
    )
    _timed(
        timings,
        "validate_inset_mesh_contract",
        lambda: validate_inset_mesh_contract(
            surface_partitions=surface_partitions,
            mesh_size_hints=mesh_size_hints,
        ),
    )
    _timed(
        timings,
        "validate_surface_sheet_interface_coverage",
        lambda: validate_surface_sheet_interface_coverage(
            build_input,
            route=route,
            surfaces=surfaces,
        ),
    )
    interfaces = _timed(
        timings,
        "complete_interface_plan_from_surfaces",
        lambda: complete_interface_plan_from_surfaces(
            interfaces=interfaces,
            surfaces=surfaces,
        ),
    )
    points, curves, surface_loops, surfaces = _timed(
        timings,
        "plan_canonical_topology",
        lambda: plan_canonical_topology(surfaces=surfaces),
    )
    _timed(
        timings,
        "validate_curve_plan_coverage",
        lambda: validate_curve_plan_coverage(
            points=points,
            curves=curves,
            surface_loops=surface_loops,
            surfaces=surfaces,
        ),
    )
    _timed(
        timings,
        "validate_surface_deduplication",
        lambda: validate_surface_deduplication(surfaces=surfaces),
    )
    _timed(
        timings,
        "validate_no_surface_overlap",
        lambda: validate_no_surface_overlap(surfaces=surfaces),
    )
    volumes = _timed(
        timings,
        "plan_route_volumes",
        lambda: plan_route_volumes(build_input, route=route, surfaces=surfaces),
    )
    _timed(
        timings,
        "validate_route_volume_surface_refs",
        lambda: validate_route_volume_surface_refs(route=route, volumes=volumes),
    )
    _timed(
        timings,
        "validate_volume_surface_closure",
        lambda: validate_volume_surface_closure(
            volumes=volumes,
            surfaces=surfaces,
            surface_loops=surface_loops,
        ),
    )
    _timed(
        timings,
        "validate_surface_use_counts",
        lambda: validate_surface_use_counts(volumes=volumes, surfaces=surfaces),
    )
    cut_operations = _timed(
        timings,
        "plan_cut_host_operations",
        lambda: plan_cut_host_operations(
            route=route,
            construction_bodies=construction_bodies,
        ),
    )
    tags = _timed(
        timings,
        "plan_route_tags",
        lambda: plan_route_tags(route=route, surfaces=surfaces, volumes=volumes),
    )
    _timed(
        timings,
        "validate_route_operation_coverage",
        lambda: validate_route_operation_coverage(
            construction_bodies=construction_bodies,
            cut_operations=cut_operations,
            surfaces=surfaces,
        ),
    )
    _timed(
        timings,
        "validate_surface_partition_coverage",
        lambda: validate_surface_partition_coverage(
            interfaces=interfaces,
            surface_partitions=surface_partitions,
            surfaces=surfaces,
        ),
    )
    _timed(
        timings,
        "validate_interface_surface_source_of_truth",
        lambda: validate_interface_surface_source_of_truth(
            interfaces=interfaces,
            surfaces=surfaces,
        ),
    )
    _timed(
        timings,
        "validate_tag_plan_coverage",
        lambda: validate_tag_plan_coverage(
            surfaces=surfaces,
            volumes=volumes,
            tags=tags,
        ),
    )
    return ConstructionPlanRecord(
        route=route,
        interfaces=interfaces,
        surface_partitions=surface_partitions,
        coplanar_inset_families=coplanar_inset_families,
        mesh_size_hints=mesh_size_hints,
        points=points,
        curves=curves,
        surface_loops=surface_loops,
        surfaces=surfaces,
        volumes=volumes,
        construction_bodies=construction_bodies,
        cut_operations=cut_operations,
        tags=tags,
        metadata={
            "backend_strategy": "surface_plan_first_bottom_up_occ",
            "fragment_first_disabled": True,
            "timings": timings,
        },
    )


def _timed(
    timings: list[dict[str, Any]],
    stage: str,
    fn: Any,
) -> Any:
    started = time.perf_counter()
    try:
        result = fn()
    except Exception:
        timings.append(
            {
                "stage": stage,
                "seconds": round(time.perf_counter() - started, 6),
                "status": "failed",
            }
        )
        raise
    timings.append(
        {
            "stage": stage,
            "seconds": round(time.perf_counter() - started, 6),
            "status": "done",
        }
    )
    return result


def complete_interface_plan_from_surfaces(
    *,
    interfaces: tuple[InterfacePlanRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> tuple[InterfacePlanRecord, ...]:
    """Backfill InterfacePlan records for planned adjacency surfaces."""
    known = {interface.interface_id for interface in interfaces}
    completed = list(interfaces)
    for surface in surfaces:
        if surface.interface_id is None or surface.interface_id in known:
            continue
        kind = surface.interface_id.split("__", 1)[0]
        if kind not in _INTERFACE_KIND_ORDER:
            raise ValueError(f"invalid surface interface id: {surface.interface_id}")
        owners = _surface_owner_ids(surface)
        record_owners = owners
        if len(record_owners) != 2:
            boundary_owners = _surface_boundary_volume_ids(
                surface,
                known_entity_ids=set(owners),
            )
            if len(boundary_owners) == 2:
                record_owners = boundary_owners
        if len(record_owners) != 2:
            raise ValueError(
                f"{surface.surface_id} interface needs two owners, got {owners!r}"
            )
        completed.append(
            InterfacePlanRecord(
                interface_id=surface.interface_id,
                kind=kind,  # type: ignore[arg-type]
                owner_semantic_ids=(record_owners[0], record_owners[1]),
                recognition_rule=str(
                    surface.metadata.get(
                        "recognition_rule",
                        "planned_surface_adjacency",
                    )
                ),
                solver_use=surface.solver_use,
                metadata={
                    "generated_from_surface_id": surface.surface_id,
                    **dict(surface.metadata),
                },
            )
        )
        known.add(surface.interface_id)
    return tuple(completed)


def plan_canonical_topology(
    *,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> tuple[
    tuple[PointPlanRecord, ...],
    tuple[CurvePlanRecord, ...],
    tuple[SurfaceLoopRecord, ...],
    tuple[SurfacePlanRecord, ...],
]:
    """Canonicalize planned surface boundaries into compiler-owned topology.

    This is the v1 implementation boundary that turns surface geometry into a
    topology registry. It must:

    - collect every outer/hole/quad boundary from planned surfaces;
    - create one `PointPlanRecord` per unique live coordinate;
    - split collinear overlapping edges and T-junctions into shared atomic
      curves;
    - create `CurvePlanRecord`s and ordered `SurfaceLoopRecord`s;
    - assign `outer_loop_ref` / `hole_loop_refs` on each surface;
    - reject parent-plus-child live overlap after inset partitioning;
    - ensure every interface surface is backed by `InterfacePlanRecord`;
    - reject duplicate live surfaces unless they are intentionally merged into
      one surface id before tagging; and
    - make volume closure checkable without asking OCC to discover topology.

    Raw `geometry_ref` lowering is not a v1 conformal-geometry contract; the
    backend consumes the planned point/curve/surface-loop refs produced here.
    """
    point_ids: dict[tuple[float, float, float], str] = {}
    point_coordinates: dict[str, tuple[float, float, float]] = {}
    point_curve_ids: dict[str, set[str]] = {}
    curve_ids: dict[tuple[str, str], str] = {}
    curve_owner_ids: dict[str, set[str]] = {}
    curve_interface_ids: dict[str, set[str]] = {}
    curve_surface_ids: dict[str, set[str]] = {}
    curve_volume_ids: dict[str, set[str]] = {}
    loop_ids: dict[tuple[str, tuple[tuple[str, int], ...]], str] = {}
    loops: dict[str, SurfaceLoopRecord] = {}
    canonical_surfaces: list[SurfacePlanRecord] = []
    surface_specs: list[
        tuple[
            SurfacePlanRecord,
            tuple[tuple[str, int, tuple[tuple[float, float, float], ...]], ...],
        ]
    ] = []

    def point_id(coordinate: tuple[float, float, float]) -> str:
        key = _coordinate_key(coordinate)
        existing = point_ids.get(key)
        if existing is not None:
            return existing
        new_id = f"P__{len(point_ids):06d}"
        point_ids[key] = new_id
        point_coordinates[new_id] = key
        point_curve_ids[new_id] = set()
        return new_id

    for surface in surfaces:
        if surface.construction_only:
            canonical_surfaces.append(surface)
            continue
        specs = _surface_ring3d_specs(surface)
        surface_specs.append((surface, specs))
        for _, _, ring in specs:
            for coordinate in ring:
                point_id(coordinate)

    point_axis_index = _point_axis_index(point_coordinates.values())
    point_line_index = _axis_aligned_point_index(point_coordinates.values())

    def split_edge_points(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], ...]:
        points_on_edge = [
            (parameter, coordinate)
            for coordinate in _segment_candidate_points(
                start,
                end,
                point_axis_index,
                point_line_index,
            )
            for parameter in (_segment_parameter(coordinate, start, end),)
            if parameter is not None
        ]
        return tuple(coordinate for _, coordinate in sorted(points_on_edge))

    def curve_ref(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        surface: SurfacePlanRecord,
    ) -> CurveRefRecord:
        start_id = point_id(start)
        end_id = point_id(end)
        if start_id == end_id:
            raise ValueError(f"{surface.surface_id} has zero-length curve")
        key = tuple(sorted((start_id, end_id)))
        curve_id = curve_ids.get(key)
        if curve_id is None:
            curve_id = f"C__{len(curve_ids):06d}"
            curve_ids[key] = curve_id
        point_curve_ids[start_id].add(curve_id)
        point_curve_ids[end_id].add(curve_id)
        curve_owner_ids.setdefault(curve_id, set()).update(
            str(owner_id)
            for owner_id in _surface_owner_ids(surface)
        )
        if surface.interface_id is not None:
            curve_interface_ids.setdefault(curve_id, set()).add(surface.interface_id)
        curve_surface_ids.setdefault(curve_id, set()).add(surface.surface_id)
        curve_volume_ids.setdefault(curve_id, set()).update(
            str(volume_id)
            for volume_id in surface.metadata.get("boundary_volume_ids", ())
        )
        return CurveRefRecord(
            curve_id=curve_id,
            orientation=1 if key == (start_id, end_id) else -1,
            role="boundary",
        )

    for surface, specs in surface_specs:
        loop_refs: list[str] = []
        for role, index, ring in specs:
            curve_refs = tuple(
                curve_ref(segment_start, segment_end, surface)
                for start, end in _ring3d_edges(ring)
                for split_points in (split_edge_points(start, end),)
                for segment_start, segment_end in zip(
                    split_points,
                    split_points[1:],
                    strict=False,
                )
            )
            loop_key = (role, _ordered_loop_signature(curve_refs))
            loop_id = loop_ids.get(loop_key)
            if loop_id is None:
                suffix = "OUTER" if role == "outer" else f"HOLE_{index:04d}"
                loop_id = f"LOOP__{surface.surface_id}__{suffix}"
                loop_ids[loop_key] = loop_id
                loops[loop_id] = SurfaceLoopRecord(
                    loop_id=loop_id,
                    curve_refs=curve_refs,
                    role=role,
                    surface_id=surface.surface_id,
                )
            loop_refs.append(loop_id)
        if not loop_refs:
            raise ValueError(f"{surface.surface_id} has no planned loops")
        canonical_surfaces.append(
            replace(
                surface,
                outer_loop_ref=loop_refs[0],
                hole_loop_refs=tuple(loop_refs[1:]),
            )
        )

    points = tuple(
        PointPlanRecord(
            point_id=point_id_,
            coordinate=point_coordinates[point_id_],
            used_by_curve_ids=tuple(sorted(point_curve_ids[point_id_])),
        )
        for point_id_ in sorted(point_coordinates)
    )
    curves = tuple(
        CurvePlanRecord(
            curve_id=curve_id,
            curve_kind="line_segment",
            start_point_id=start_id,
            end_point_id=end_id,
            owner_semantic_ids=tuple(sorted(curve_owner_ids.get(curve_id, ()))),
            interface_ids=tuple(sorted(curve_interface_ids.get(curve_id, ()))),
            used_by_surface_ids=tuple(sorted(curve_surface_ids.get(curve_id, ()))),
            boundary_volume_ids=tuple(sorted(curve_volume_ids.get(curve_id, ()))),
        )
        for (start_id, end_id), curve_id in sorted(
            curve_ids.items(),
            key=lambda item: item[1],
        )
    )
    return points, curves, tuple(loops.values()), tuple(canonical_surfaces)


def _surface_ring3d_specs(
    surface: SurfacePlanRecord,
) -> tuple[tuple[str, int, tuple[tuple[float, float, float], ...]], ...]:
    geometry_ref = surface.geometry_ref
    if "quad_points" in geometry_ref:
        return (("outer", 0, _clean_ring3d(geometry_ref["quad_points"])),)
    if "outer_loop" not in geometry_ref:
        raise ValueError(f"{surface.surface_id} requires outer_loop or quad_points")
    z_um = _geometry_ref_surface_z_um(geometry_ref)
    outer_loop = _canonical_planar_loop_orientation(
        _clean_loop(geometry_ref["outer_loop"])
    )
    specs = [
        (
            "outer",
            0,
            tuple((x, y, z_um) for x, y in outer_loop),
        )
    ]
    specs.extend(
        (
            "hole",
            index,
            tuple(
                (x, y, z_um)
                for x, y in _canonical_planar_loop_orientation(
                    _clean_loop(hole_loop)
                )
            ),
        )
        for index, hole_loop in enumerate(geometry_ref.get("hole_loops", ()))
    )
    return tuple(specs)


def _canonical_planar_loop_orientation(
    loop: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    """Use one XY loop direction for OCC plane surfaces and their holes."""
    if _polygon_area(loop) < 0:
        return tuple(reversed(loop))
    return loop


def _geometry_ref_surface_z_um(geometry_ref: Mapping[str, Any]) -> float:
    z_min_um = float(geometry_ref.get("z_min_um", geometry_ref.get("z_um", 0.0)))
    if geometry_ref.get("shell_part") == "top":
        return z_min_um + float(geometry_ref.get("thickness_um", 0.0))
    if geometry_ref.get("shell_part") == "bottom":
        return z_min_um
    plane = geometry_ref.get("plane") or geometry_ref.get("contact_plane")
    if isinstance(plane, Mapping) and plane.get("axis") == "z":
        return float(plane["value_um"])
    return z_min_um


def _clean_ring3d(ring: Any) -> tuple[tuple[float, float, float], ...]:
    points = tuple(
        (float(point[0]), float(point[1]), float(point[2]))
        for point in ring
    )
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("3D loop requires at least 3 unique points")
    return points


def _ring3d_edges(
    ring: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...]:
    return tuple(
        (ring[index], ring[(index + 1) % len(ring)])
        for index in range(len(ring))
    )


def _coordinate_key(
    coordinate: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(round(float(value), 9) for value in coordinate)


def _segment_parameter(
    point: tuple[float, float, float],
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float | None:
    vector = tuple(end[index] - start[index] for index in range(3))
    offset = tuple(point[index] - start[index] for index in range(3))
    length_sq = sum(value * value for value in vector)
    if length_sq <= 1e-18:
        return None
    parameter = sum(offset[index] * vector[index] for index in range(3)) / length_sq
    if parameter < -1e-9 or parameter > 1.0 + 1e-9:
        return None
    closest = tuple(start[index] + parameter * vector[index] for index in range(3))
    distance_sq = sum((point[index] - closest[index]) ** 2 for index in range(3))
    if distance_sq > 1e-18:
        return None
    return max(0.0, min(1.0, parameter))


def _point_axis_index(
    coordinates: Sequence[tuple[float, float, float]],
) -> tuple[
    tuple[tuple[float, ...], tuple[tuple[float, float, float], ...]],
    ...,
]:
    indexes: list[tuple[tuple[float, ...], tuple[tuple[float, float, float], ...]]] = []
    for axis in range(3):
        items = sorted((coordinate[axis], coordinate) for coordinate in coordinates)
        indexes.append(
            (
                tuple(value for value, _ in items),
                tuple(coordinate for _, coordinate in items),
            )
        )
    return tuple(indexes)


def _axis_aligned_point_index(
    coordinates: Sequence[tuple[float, float, float]],
) -> dict[
    tuple[int, tuple[float, float]],
    tuple[tuple[float, ...], tuple[tuple[float, float, float], ...]],
]:
    records: dict[
        tuple[int, tuple[float, float]],
        list[tuple[float, tuple[float, float, float]]],
    ] = {}
    for coordinate in coordinates:
        key = _coordinate_key(coordinate)
        for varying_axis in range(3):
            fixed_key = tuple(
                key[axis]
                for axis in range(3)
                if axis != varying_axis
            )
            records.setdefault((varying_axis, fixed_key), []).append(
                (key[varying_axis], key)
            )
    return {
        line_key: (
            tuple(value for value, _ in sorted_items),
            tuple(coordinate for _, coordinate in sorted_items),
        )
        for line_key, items in records.items()
        for sorted_items in (tuple(sorted(items)),)
    }


def _segment_candidate_points(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    point_axis_index: Sequence[
        tuple[tuple[float, ...], tuple[tuple[float, float, float], ...]]
    ],
    point_line_index: Mapping[
        tuple[int, tuple[float, float]],
        tuple[tuple[float, ...], tuple[tuple[float, float, float], ...]],
    ],
) -> tuple[tuple[float, float, float], ...]:
    start_key = _coordinate_key(start)
    end_key = _coordinate_key(end)
    varying_axes = tuple(
        axis
        for axis in range(3)
        if abs(start_key[axis] - end_key[axis]) > _INSET_EPS_UM
    )
    if len(varying_axes) == 1:
        varying_axis = varying_axes[0]
        fixed_key = tuple(
            start_key[axis]
            for axis in range(3)
            if axis != varying_axis
        )
        values, coordinates = point_line_index.get(
            (varying_axis, fixed_key),
            ((), ()),
        )
        lower = min(start_key[varying_axis], end_key[varying_axis]) - _INSET_EPS_UM
        upper = max(start_key[varying_axis], end_key[varying_axis]) + _INSET_EPS_UM
        left = bisect_left(values, lower)
        right = bisect_right(values, upper)
        candidates = set(coordinates[left:right])
        candidates.update((start_key, end_key))
        return tuple(sorted(candidates))
    bounds = tuple(
        (
            min(start_key[axis], end_key[axis]) - _INSET_EPS_UM,
            max(start_key[axis], end_key[axis]) + _INSET_EPS_UM,
        )
        for axis in range(3)
    )
    ranges: list[tuple[int, int, int]] = []
    for axis, (values, _) in enumerate(point_axis_index):
        lower, upper = bounds[axis]
        left = bisect_left(values, lower)
        right = bisect_right(values, upper)
        ranges.append((right - left, left, axis))
    _, left, axis = min(ranges)
    right = left + min(ranges)[0]
    coordinates = point_axis_index[axis][1][left:right]
    candidates = {
        coordinate
        for coordinate in coordinates
        if all(
            bounds[coordinate_axis][0]
            <= coordinate[coordinate_axis]
            <= bounds[coordinate_axis][1]
            for coordinate_axis in range(3)
        )
    }
    candidates.update((start_key, end_key))
    return tuple(sorted(candidates))


def _surface_owner_ids(surface: SurfacePlanRecord) -> tuple[str, ...]:
    owner_ids = surface.metadata.get("owner_semantic_ids", (surface.owner_semantic_id,))
    if isinstance(owner_ids, str):
        return (owner_ids,)
    if isinstance(owner_ids, Sequence):
        return tuple(str(owner_id) for owner_id in owner_ids)
    return (surface.owner_semantic_id,)


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


def plan_conductor_contact_patches(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interfaces: tuple[InterfacePlanRecord, ...],
) -> tuple[InterfacePlanRecord, ...]:
    """Recognize coplanar conductor contact patches from planned conductors.

    This is the small v1 contact planner: it only accepts one rectangular
    overlap polygon per opposite face pair. Bboxes are used only to skip
    impossible pairs; the contact geometry itself comes from gdstk boolean
    intersection.
    """
    import gdstk

    generated: list[InterfacePlanRecord] = []
    seen = {_contact_signature(interface) for interface in interfaces}
    solution_entities = _solution_entities(build_input)
    top_faces: dict[
        float,
        list[tuple[SemanticEntitySpec, Any, Mapping[str, float]]],
    ] = {}
    bottom_faces: dict[
        float,
        list[tuple[SemanticEntitySpec, Any, Mapping[str, float]]],
    ] = {}

    for entity in _active_route_conductor_entities(build_input, route):
        region = _entity_occupied_region(gdstk, entity)
        if not region:
            continue
        bounds = _entity_loop_bounds(entity)
        z_min_um, z_max_um = _entity_z_range_um(entity)
        bottom_faces.setdefault(_z_key(z_min_um), []).append((entity, region, bounds))
        top_faces.setdefault(_z_key(z_max_um), []).append((entity, region, bounds))

    index = 0
    for z_key, lower_faces in top_faces.items():
        for lower, lower_region, lower_bounds in lower_faces:
            for upper, upper_region, upper_bounds in bottom_faces.get(z_key, ()):
                if lower.semantic_id == upper.semantic_id:
                    continue
                if not _bounds_overlap(lower_bounds, upper_bounds):
                    continue
                overlap_region = _boolean_gdstk_region(
                    gdstk,
                    lower_region,
                    upper_region,
                    "and",
                )
                if not overlap_region:
                    continue
                contact_loop = _single_rectangular_contact_loop(
                    overlap_region,
                    lower.semantic_id,
                    upper.semantic_id,
                )
                signature = (
                    lower.semantic_id,
                    upper.semantic_id,
                    _z_key(float(z_key)),
                    _loop_signature(contact_loop),
                )
                if signature in seen:
                    continue
                seen.add(signature)
                metadata = _contact_patch_metadata(
                    build_input,
                    route=route,
                    solution_entities=solution_entities,
                    lower=lower,
                    upper=upper,
                    contact_z_um=float(z_key),
                    contact_loop=contact_loop,
                )
                generated.append(
                    InterfacePlanRecord(
                        interface_id=(
                            f"MM__CONTACT__{lower.semantic_id}__"
                            f"{upper.semantic_id}__{index:04d}"
                        ),
                        kind="MM",
                        owner_semantic_ids=(lower.semantic_id, upper.semantic_id),
                        recognition_rule="coplanar_conductor_contact_patch",
                        source_polygon_ids=(
                            *lower.polygon_ids,
                            *upper.polygon_ids,
                        ),
                        metadata=metadata,
                    )
                )
                index += 1
    return (*interfaces, *generated)


def plan_surface_partitions(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interfaces: tuple[InterfacePlanRecord, ...],
) -> tuple[SurfacePartitionRecord, ...]:
    """Plan parent-interface partitions before live surfaces are created.

    This function only consumes explicit `build_input.metadata["surface_partitions"]`
    records. The normal tutorial inset path is `apply_inset_surface_partitions()`,
    which creates child surfaces after route surfaces exist so it can partition
    the actual live parent surface geometry.

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

    The v1 target is coplanar-family partitioning: all inset-eligible
    `SA`/`MS`/`MA`/`MM`/`SS`/`AA` parents on the same plane that touch the same
    volume boundary must be partitioned from one shared planar topology graph.
    This preserves shared points/curves between adjacent child surfaces and
    prevents the `SA` side and `MS` side of a metal edge from inventing
    independent near-duplicate micro topology.

    The current implementation still performs the low-risk fallback for
    isolated parents: planar surfaces use real 2D inward offsets; sidewall
    surfaces use sidewall-specific strip partitioning from both sides of the
    shorter local axis. Requested thresholds that are not strictly smaller than
    half of that axis length are skipped, and the remaining middle area becomes
    `CORE`.
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
        inset_family_ids = _inset_family_ids(surface)
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
                "inset_family_ids": inset_family_ids,
                "inset_partition_source": "surface_local_fallback",
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
                        "inset_family_ids": inset_family_ids,
                        "inset_partition_source": "surface_local_fallback",
                        "parent_surface_id": surface.surface_id,
                    },
                )
            )
    return tuple(child_surfaces), tuple(partitions)


def plan_coplanar_inset_families(
    *,
    surfaces: tuple[SurfacePlanRecord, ...],
    surface_partitions: tuple[SurfacePartitionRecord, ...],
    breakpoints_um: tuple[float, ...],
) -> tuple[CoplanarInsetFamilyRecord, ...]:
    """Collect shared-plane inset ownership into first-class plan records."""
    partitions_by_child = {
        partition.child_surface_id: partition
        for partition in surface_partitions
    }
    families: dict[str, dict[str, Any]] = {}
    for surface in surfaces:
        if surface.parent_surface_id is None:
            continue
        partition = partitions_by_child.get(surface.surface_id)
        for family_id in surface.metadata.get("inset_family_ids", ()):
            family_key = str(family_id)
            entry = families.setdefault(
                family_key,
                {
                    "parent_surface_ids": set(),
                    "child_surface_ids": set(),
                    "partition_ids": set(),
                    "sources": set(),
                },
            )
            entry["parent_surface_ids"].add(surface.parent_surface_id)
            entry["child_surface_ids"].add(surface.surface_id)
            if partition is not None:
                entry["partition_ids"].add(partition.partition_id)
            source = surface.metadata.get(
                "inset_partition_source",
                "surface_local_fallback",
            )
            entry["sources"].add(str(source))

    records: list[CoplanarInsetFamilyRecord] = []
    for family_id, entry in sorted(families.items()):
        if len(entry["sources"]) != 1:
            raise ValueError(
                f"{family_id} has mixed inset partition sources "
                f"{sorted(entry['sources'])!r}"
            )
        plane_key, boundary_volume_id = _parse_inset_family_id(family_id)
        source = next(iter(entry["sources"]))
        records.append(
            CoplanarInsetFamilyRecord(
                family_id=family_id,
                plane_key=plane_key,
                boundary_volume_id=boundary_volume_id,
                parent_surface_ids=tuple(sorted(entry["parent_surface_ids"])),
                breakpoints_um=breakpoints_um,
                source=source,  # type: ignore[arg-type]
                child_surface_ids=tuple(sorted(entry["child_surface_ids"])),
                metadata={
                    "source_partition_ids": tuple(sorted(entry["partition_ids"])),
                },
            )
        )
    return tuple(records)


def plan_mesh_size_hints(
    *,
    surface_partitions: tuple[SurfacePartitionRecord, ...],
) -> tuple[MeshSizeHintRecord, ...]:
    """Export mesh-size hints implied by finite inset bands."""
    hints: list[MeshSizeHintRecord] = []
    for partition in surface_partitions:
        if partition.band_min_um is None or partition.band_max_um is None:
            continue
        width_um = partition.band_max_um - partition.band_min_um
        if width_um <= 0:
            continue
        hints.append(
            MeshSizeHintRecord(
                target_id=partition.child_surface_id,
                max_size_um=width_um / 2.0,
                reason="inset_band_width",
                source_partition_id=partition.partition_id,
                metadata={
                    "band_min_um": partition.band_min_um,
                    "band_max_um": partition.band_max_um,
                    "required_elements_across_band": 2,
                },
            )
        )
    return tuple(hints)


def validate_coplanar_inset_family_contract(
    *,
    surfaces: tuple[SurfacePlanRecord, ...],
    coplanar_inset_families: tuple[CoplanarInsetFamilyRecord, ...],
) -> None:
    """Refuse local inset when one coplanar family spans multiple parents."""
    source_by_family = {
        family.family_id: family.source
        for family in coplanar_inset_families
    }
    family_parent_bounds: dict[str, dict[str, dict[str, float]]] = {}
    for surface in surfaces:
        parent_id = surface.parent_surface_id
        if parent_id is None:
            continue
        for family_id in surface.metadata.get("inset_family_ids", ()):
            source = source_by_family.get(str(family_id))
            if source is None:
                raise ValueError(f"{surface.surface_id} references unknown {family_id}")
            if source != "coplanar_joint_arrangement":
                family = family_parent_bounds.setdefault(str(family_id), {})
                bounds = _surface_planar_bounds(surface)
                existing = family.get(parent_id)
                family[parent_id] = (
                    bounds
                    if existing is None
                    else _merge_bounds(existing, bounds)
                )
    invalid: dict[str, list[str]] = {}
    for family_id, parent_bounds in family_parent_bounds.items():
        parent_ids = sorted(parent_bounds)
        for index, left_id in enumerate(parent_ids):
            for right_id in parent_ids[index + 1 :]:
                if _bounds_touch_or_overlap(
                    parent_bounds[left_id],
                    parent_bounds[right_id],
                ):
                    invalid.setdefault(family_id, []).extend((left_id, right_id))
                    break
            if family_id in invalid:
                invalid[family_id] = sorted(set(invalid[family_id]))
                break
    if invalid:
        raise NotImplementedError(
            "coplanar inset requires joint plane partitioning; local "
            f"per-surface offsets are not conformal enough for {invalid!r}"
        )


def plan_route_construction_bodies(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interfaces: tuple[InterfacePlanRecord, ...] = (),
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
    contact_faces = _contact_patches_by_entity_face(interfaces)
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
                geometry_ref=_route_entity_geometry_ref(
                    build_input,
                    route,
                    entity,
                    representation=representation,
                    interfaces=interfaces,
                ),
                expected_surface_ids=_cutout_shell_surface_ids(
                    build_input,
                    route,
                    entity,
                    interfaces=interfaces,
                    contact_faces=contact_faces,
                ),
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
    contact_faces = _contact_patches_by_entity_face(interfaces)
    for interface in interfaces:
        if _is_hidden_contact_interface(route, interface):
            continue
        geometry_ref = {
            "from_interface_id": interface.interface_id,
            **_geometry_ref_from_metadata(interface.metadata),
        }
        surface_interface_id = _surface_interface_id(
            build_input,
            route=route,
            interface=interface,
        )
        if _is_route_a_sheet_interface(route, interface):
            sheet_entity = _entity_by_id(
                build_input,
                interface.owner_semantic_ids[0],
            )
            geometry_ref = {
                **geometry_ref,
                "plane": {
                    "axis": "z",
                    "value_um": _route_a_sheet_plane_z_um(
                        build_input,
                        sheet_entity,
                    ),
                },
            }
            sheet_contact_loops = _contact_loops_for_entity(
                contact_faces,
                sheet_entity.semantic_id,
            )
            if sheet_contact_loops:
                geometry_refs = _subtract_contact_patches_from_face(
                    geometry_ref,
                    sheet_contact_loops,
                )
                if len(geometry_refs) != 1:
                    raise ValueError(
                        f"{sheet_entity.semantic_id} Route A sheet contact "
                        "split produced multiple sheet remainders"
                    )
                geometry_ref = geometry_refs[0]
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
        boundary_volume_ids = _interface_boundary_volume_ids(
            build_input,
            route=route,
            interface=interface,
        )
        physical_owner_semantic_ids = _physical_group_owner_ids(
            build_input,
            owner_semantic_ids,
        )
        partitions = partitions_by_interface.get(interface.interface_id, ())
        if partitions:
            parent_surface_id = f"SURF__{surface_interface_id}"
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
                        interface_id=surface_interface_id,
                        parent_surface_id=parent_surface_id,
                        partition_label=partition.label,
                        solver_use=interface.solver_use or "solver_active",
                        valid_routes=(route,),
                        metadata={
                            "interface_kinds": interface_kinds,
                            "owner_semantic_ids": owner_semantic_ids,
                            "physical_owner_semantic_ids": physical_owner_semantic_ids,
                            "boundary_volume_ids": boundary_volume_ids,
                        },
                    )
                )
            continue
        records.append(
            SurfacePlanRecord(
                surface_id=f"SURF__{surface_interface_id}",
                owner_semantic_id=interface.owner_semantic_ids[0],
                surface_role=f"{route}_planned_interface",
                geometry_ref=geometry_ref,
                interface_id=surface_interface_id,
                solver_use=interface.solver_use or "solver_active",
                valid_routes=(route,),
                metadata={
                    "interface_kinds": interface_kinds,
                    "owner_semantic_ids": owner_semantic_ids,
                    "physical_owner_semantic_ids": physical_owner_semantic_ids,
                    "boundary_volume_ids": boundary_volume_ids,
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
        if representation in {"cutout_boundary_shell", "material_volume"}:
            for shell_part in ("top", "bottom"):
                adjacent_id = _conductor_face_adjacent_solution_id(
                    build_input,
                    entity,
                    shell_part,
                )
                interface_kind = _conductor_solution_interface_kind(
                    _entity_by_id(build_input, adjacent_id)
                )
                base_surface_id = _conductor_boundary_surface_id(
                    route,
                    entity,
                    representation,
                    shell_part,
                )
                base_geometry_ref = {
                    **_route_entity_geometry_ref(
                        build_input,
                        route,
                        entity,
                        representation=representation,
                        interfaces=interfaces,
                    ),
                    "shell_part": shell_part,
                }
                face_geometry_refs = _subtract_contact_patches_from_face(
                    base_geometry_ref,
                    contact_faces.get((entity.semantic_id, shell_part), ()),
                )
                for face_index, face_geometry_ref in enumerate(face_geometry_refs):
                    surface_id = (
                        base_surface_id
                        if len(face_geometry_refs) == 1
                        else f"{base_surface_id}__P{face_index:04d}"
                    )
                    body = construction_body_by_surface_id.get(surface_id)
                    if body is None:
                        body = construction_body_by_surface_id.get(base_surface_id)
                    face_owner_ids = (entity.semantic_id, adjacent_id)
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
                                **face_geometry_ref,
                                "construction_body_id": (
                                    body.construction_body_id
                                    if body is not None
                                    else None
                                ),
                            },
                            interface_id=_conductor_face_interface_id(
                                interface_kind,
                                entity.semantic_id,
                                adjacent_id,
                                shell_part,
                                None
                                if len(face_geometry_refs) == 1
                                else face_index,
                            ),
                            valid_routes=(route,),
                            solver_use="solver_active",
                            metadata={
                                "interface_kinds": (interface_kind,),
                                "owner_semantic_ids": face_owner_ids,
                                "physical_owner_semantic_ids": (
                                    _physical_group_owner_ids(
                                        build_input,
                                        face_owner_ids,
                                    )
                                ),
                                "boundary_volume_ids": _conductor_boundary_volume_ids(
                                    route,
                                    entity,
                                    adjacent_id,
                                ),
                                "exposed_surface_role": shell_part,
                            },
                        )
                    )
            sidewall_adjacent_id = _conductor_sidewall_adjacent_solution_id(
                build_input,
                entity,
            )
            sidewall_geometry_refs = _conductor_sidewall_geometry_refs(
                build_input,
                route=route,
                entity=entity,
                representation=representation,
                adjacent_solution_id=sidewall_adjacent_id,
                interfaces=interfaces,
            )
            for edge_index, geometry_ref in enumerate(sidewall_geometry_refs):
                shell_part = f"sidewall_{edge_index:04d}"
                surface_id = _conductor_boundary_surface_id(
                    route,
                    entity,
                    representation,
                    shell_part,
                )
                body = construction_body_by_surface_id.get(surface_id)
                sidewall_adjacent_owner_id = str(
                    geometry_ref.get(
                        "adjacent_conductor_semantic_id",
                        sidewall_adjacent_id,
                    )
                )
                sidewall_interface_kind = (
                    "MM"
                    if "adjacent_conductor_semantic_id" in geometry_ref
                    else _conductor_solution_interface_kind(
                        _entity_by_id(build_input, sidewall_adjacent_id)
                    )
                )
                sidewall_interface_id = (
                    None
                    if geometry_ref.get("solution_exterior_boundary")
                    else (
                        f"{sidewall_interface_kind}__{entity.semantic_id}__"
                        f"{sidewall_adjacent_owner_id}__"
                        f"{shell_part.upper()}"
                    )
                )
                sidewall_boundary_volume_ids = (
                    (entity.semantic_id,)
                    if geometry_ref.get("solution_exterior_boundary")
                    else _conductor_boundary_volume_ids(
                        route,
                        entity,
                        sidewall_adjacent_owner_id,
                    )
                )
                sidewall_owner_ids = (
                    (entity.semantic_id,)
                    if geometry_ref.get("solution_exterior_boundary")
                    else (entity.semantic_id, sidewall_adjacent_owner_id)
                )
                sidewall_physical_owner_ids = _physical_group_owner_ids(
                    build_input,
                    sidewall_owner_ids,
                )
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
                        interface_id=sidewall_interface_id,
                        valid_routes=(route,),
                        solver_use="solver_active",
                        metadata={
                            "interface_kinds": (
                                ()
                                if sidewall_interface_id is None
                                else (sidewall_interface_kind,)
                            ),
                            "owner_semantic_ids": sidewall_owner_ids,
                            "physical_owner_semantic_ids": (
                                sidewall_physical_owner_ids
                            ),
                            "boundary_volume_ids": sidewall_boundary_volume_ids,
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
    entity_ids = {entity.semantic_id for entity in build_input.entities}
    surfaces_by_owner: dict[str, list[SurfacePlanRecord]] = {}
    for surface in surfaces:
        owner_ids = _surface_boundary_volume_ids(
            surface,
            known_entity_ids=entity_ids,
        )
        for owner_id in owner_ids:
            surfaces_by_owner.setdefault(str(owner_id), []).append(surface)

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
                            surface_id=surface.surface_id,
                            orientation=_surface_orientation_for_volume(
                                surface,
                                entity,
                            ),
                            role="planned_boundary",
                        )
                        for surface in surfaces_by_owner.get(
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
                            surface_id=surface.surface_id,
                            orientation=_surface_orientation_for_volume(
                                surface,
                                entity,
                            ),
                            role="planned_boundary",
                        )
                        for surface in surfaces_by_owner.get(
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
                        "physical_owner_semantic_id": _entity_physical_group_id(
                            entity,
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

    This is semantic grouping and provenance in the current v1 backend. Exposed
    shell surfaces are already planned as `SurfacePlanRecord`s; these records
    explain which construction bodies belong to each host exclusion policy
    without asking the backend to discover new surfaces through boolean cuts.
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
            physical_name=_volume_physical_name(volume),
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


def _contact_signature(interface: InterfacePlanRecord) -> tuple[Any, ...]:
    if interface.recognition_rule != "coplanar_conductor_contact_patch":
        return ()
    return (
        str(interface.metadata.get("lower_entity_id", "")),
        str(interface.metadata.get("upper_entity_id", "")),
        _z_key(float(interface.metadata.get("contact_z_um", 0.0))),
        _loop_signature(interface.metadata.get("outer_loop", ())),
    )


def _active_route_conductor_entities(
    build_input: GeometryBuildInput,
    route: RouteLiteral,
) -> tuple[SemanticEntitySpec, ...]:
    return tuple(
        entity
        for entity in build_input.entities
        if not _is_solution_entity(entity)
        and entity.route_representations.get(route) is not None
        and "outer_loop" in entity.geometry
    )


def _entity_loop_bounds(entity: SemanticEntitySpec) -> Mapping[str, float]:
    points = tuple(
        point
        for loop in (
            entity.geometry["outer_loop"],
            *entity.geometry.get("hole_loops", ()),
        )
        for point in _clean_loop(loop)
    )
    return {
        "x_min_um": min(point[0] for point in points),
        "y_min_um": min(point[1] for point in points),
        "x_max_um": max(point[0] for point in points),
        "y_max_um": max(point[1] for point in points),
    }


def _bounds_overlap(
    first: Mapping[str, float],
    second: Mapping[str, float],
) -> bool:
    return not (
        float(first["x_max_um"]) <= float(second["x_min_um"]) + _INSET_EPS_UM
        or float(second["x_max_um"]) <= float(first["x_min_um"]) + _INSET_EPS_UM
        or float(first["y_max_um"]) <= float(second["y_min_um"]) + _INSET_EPS_UM
        or float(second["y_max_um"]) <= float(first["y_min_um"]) + _INSET_EPS_UM
    )


def _z_key(value_um: float) -> float:
    return round(float(value_um), 9)


def _single_rectangular_contact_loop(
    overlap_region: Sequence[Any],
    lower_id: str,
    upper_id: str,
) -> tuple[tuple[float, float], ...]:
    import gdstk

    polygons = _boolean_gdstk_region(gdstk, overlap_region, (), "or")
    if len(polygons) != 1:
        raise ValueError(
            f"{lower_id} and {upper_id} contact must produce one polygon, got "
            f"{len(polygons)}"
        )
    loop = _clean_loop(polygons[0].points)
    if not _is_axis_aligned_rectangle(loop):
        raise ValueError(
            f"{lower_id} and {upper_id} contact is not a rectangular patch"
        )
    return loop


def _is_axis_aligned_rectangle(loop: tuple[tuple[float, float], ...]) -> bool:
    if len(loop) != 4:
        return False
    return all(
        _same_z(start[0], end[0]) or _same_z(start[1], end[1])
        for start, end in _ring_edges(loop)
    )


def _loop_signature(loop: Any) -> tuple[tuple[float, float], ...]:
    try:
        points = _clean_loop(loop)
    except Exception:
        return ()
    return tuple(sorted(_coordinate_2d_key(point) for point in points))


def _contact_patch_metadata(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    solution_entities: Sequence[SemanticEntitySpec],
    lower: SemanticEntitySpec,
    upper: SemanticEntitySpec,
    contact_z_um: float,
    contact_loop: tuple[tuple[float, float], ...],
) -> Mapping[str, Any]:
    plane_z_um = contact_z_um
    boundary_volume_ids: tuple[str, ...]
    interface_kinds: tuple[str, ...] = ("MM",)
    export_surface = route != "B"

    if route == "A":
        sheet = next(
            (
                entity
                for entity in (lower, upper)
                if entity.route_representations.get(route) == "surface_sheet"
            ),
            None,
        )
        if sheet is not None:
            plane_z_um = _route_a_sheet_plane_z_um_from_solutions(
                sheet,
                solution_entities,
            )
            boundary_volume_ids = _route_a_sheet_boundary_volume_ids_from_solutions(
                sheet,
                solution_entities,
            )
            interface_kinds = ("MM", "MS")
        else:
            boundary_volume_ids = ()
            export_surface = False
    elif route == "C":
        boundary_volume_ids = (lower.semantic_id, upper.semantic_id)
    else:
        boundary_volume_ids = ()

    return {
        "recognition_rule": "coplanar_conductor_contact_patch",
        "contact_policy": {
            "A": "sheet_contact_patch",
            "B": "hidden_cutout_contact",
            "C": "retained_material_contact",
        }[route],
        "export_surface": export_surface,
        "lower_entity_id": lower.semantic_id,
        "upper_entity_id": upper.semantic_id,
        "lower_face": "top",
        "upper_face": "bottom",
        "contact_z_um": contact_z_um,
        "contact_plane": {"axis": "z", "value_um": plane_z_um},
        "plane": {"axis": "z", "value_um": plane_z_um},
        "outer_loop": contact_loop,
        "hole_loops": (),
        "interface_kinds": interface_kinds,
        "surface_owner_semantic_ids": (lower.semantic_id, upper.semantic_id),
        "boundary_volume_ids": boundary_volume_ids,
        "valid_routes": (route,),
    }


def _route_a_sheet_plane_z_um_from_solutions(
    entity: SemanticEntitySpec,
    solution_entities: Sequence[SemanticEntitySpec],
) -> float:
    z_min_um, z_max_um = _entity_z_range_um(entity)
    point = _loop_centroid(entity.geometry["outer_loop"])
    for face_z_um, solution_edge_key in (
        (z_min_um, "z_max_um"),
        (z_max_um, "z_min_um"),
    ):
        for solution in solution_entities:
            if not _bounds_contains_point(_solution_bounds(solution), point):
                continue
            if not _same_z(float(solution.geometry[solution_edge_key]), face_z_um):
                continue
            if not _is_air_like_solution_entity(solution):
                return face_z_um
    return z_min_um


def _route_a_sheet_boundary_volume_ids_from_solutions(
    entity: SemanticEntitySpec,
    solution_entities: Sequence[SemanticEntitySpec],
) -> tuple[str, ...]:
    z_min_um, z_max_um = _entity_z_range_um(entity)
    point = _loop_centroid(entity.geometry["outer_loop"])
    ids: list[str] = []
    for face_z_um, solution_edge_key in (
        (z_min_um, "z_max_um"),
        (z_max_um, "z_min_um"),
    ):
        for solution in solution_entities:
            if not _bounds_contains_point(_solution_bounds(solution), point):
                continue
            if _same_z(float(solution.geometry[solution_edge_key]), face_z_um):
                ids.append(solution.semantic_id)
                break
    return _unique_ids(ids)


def _contact_patches_by_entity_face(
    interfaces: tuple[InterfacePlanRecord, ...],
) -> dict[tuple[str, str], tuple[tuple[tuple[float, float], ...], ...]]:
    records: dict[tuple[str, str], list[tuple[tuple[float, float], ...]]] = {}
    for interface in interfaces:
        if interface.recognition_rule != "coplanar_conductor_contact_patch":
            continue
        loop = _clean_loop(interface.metadata["outer_loop"])
        lower_id = str(interface.metadata["lower_entity_id"])
        upper_id = str(interface.metadata["upper_entity_id"])
        lower_face = str(interface.metadata.get("lower_face", "top"))
        upper_face = str(interface.metadata.get("upper_face", "bottom"))
        records.setdefault((lower_id, lower_face), []).append(loop)
        records.setdefault((upper_id, upper_face), []).append(loop)
    return {key: tuple(value) for key, value in records.items()}


def _contact_loops_for_entity(
    contact_faces: Mapping[
        tuple[str, str],
        tuple[tuple[tuple[float, float], ...], ...],
    ],
    semantic_id: str,
) -> tuple[tuple[tuple[float, float], ...], ...]:
    return tuple(
        loop
        for (entity_id, _), loops in contact_faces.items()
        if entity_id == semantic_id
        for loop in loops
    )


def _subtract_contact_patches_from_face(
    geometry_ref: Mapping[str, Any],
    contact_loops: Sequence[tuple[tuple[float, float], ...]],
) -> tuple[dict[str, Any], ...]:
    if not contact_loops:
        return (dict(geometry_ref),)
    outer_loop = _clean_loop(geometry_ref["outer_loop"])
    existing_holes = tuple(
        _clean_loop(hole_loop)
        for hole_loop in geometry_ref.get("hole_loops", ())
    )
    remaining_holes: list[tuple[tuple[float, float], ...]] = list(existing_holes)
    for contact_loop in contact_loops:
        if _same_loop_geometry(contact_loop, outer_loop):
            return ()
        remaining_holes.append(_clean_loop(contact_loop))
    simple_ref = {
        **dict(geometry_ref),
        "hole_loops": tuple(remaining_holes),
        "contact_hole_loops": tuple(_clean_loop(loop) for loop in contact_loops),
    }
    if _contact_holes_are_simple(outer_loop, remaining_holes):
        return (simple_ref,)

    import gdstk

    live_region = _boolean_gdstk_region(
        gdstk,
        _gdstk_surface_region(geometry_ref),
        tuple(gdstk.Polygon(loop) for loop in contact_loops),
        "not",
    )
    return _geometry_refs_from_gdstk_region(geometry_ref, live_region)


def _same_loop_geometry(left: Any, right: Any) -> bool:
    return _loop_signature(left) == _loop_signature(right)


def _contact_holes_are_simple(
    outer_loop: tuple[tuple[float, float], ...],
    hole_loops: Sequence[tuple[tuple[float, float], ...]],
) -> bool:
    for hole_loop in hole_loops:
        if not _loop_inside_loop(hole_loop, outer_loop):
            return False
    hole_records = tuple(
        (_loop_bounds_tuple(hole_loop), hole_loop)
        for hole_loop in hole_loops
    )
    for index, (left_bounds, left) in enumerate(hole_records):
        for right_bounds, right in hole_records[index + 1 :]:
            if not _bounds_tuple_may_overlap(left_bounds, right_bounds):
                continue
            if _loops_share_edge_overlap(left, right):
                return False
    return True


def _loop_bounds_tuple(
    loop: tuple[tuple[float, float], ...],
) -> tuple[float, float, float, float]:
    return (
        min(point[0] for point in loop),
        min(point[1] for point in loop),
        max(point[0] for point in loop),
        max(point[1] for point in loop),
    )


def _bounds_tuple_may_overlap(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> bool:
    """Cheap candidate filter; contact/hole validity is checked on loops."""
    return not (
        left[2] <= right[0] + _INSET_EPS_UM
        or right[2] <= left[0] + _INSET_EPS_UM
        or left[3] <= right[1] + _INSET_EPS_UM
        or right[3] <= left[1] + _INSET_EPS_UM
    )


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
    raw_kinds = interface.metadata.get("interface_kinds")
    if raw_kinds is not None:
        if isinstance(raw_kinds, str):
            raw_kinds = (raw_kinds,)
        return tuple(kind for kind in _INTERFACE_KIND_ORDER if kind in set(raw_kinds))
    if _is_route_a_sheet_interface(route, interface):
        return ("MS", "MA")
    kinds = set(_interface_kinds(interface))
    return tuple(kind for kind in _INTERFACE_KIND_ORDER if kind in kinds)


def _interface_surface_owner_ids(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> tuple[str, ...]:
    raw_owner_ids = interface.metadata.get("surface_owner_semantic_ids")
    if raw_owner_ids is not None:
        if isinstance(raw_owner_ids, str):
            return (raw_owner_ids,)
        return _unique_ids(raw_owner_ids)
    if _is_route_a_sheet_interface(route, interface):
        entity = _entity_by_id(build_input, interface.owner_semantic_ids[0])
        return _unique_ids(
            (
                entity.semantic_id,
                *_route_a_sheet_boundary_volume_ids(build_input, entity),
            )
        )
    return tuple(interface.owner_semantic_ids)


def _interface_boundary_volume_ids(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> tuple[str, ...]:
    raw_boundary_ids = interface.metadata.get("boundary_volume_ids")
    if raw_boundary_ids is not None:
        if isinstance(raw_boundary_ids, str):
            return (raw_boundary_ids,)
        return _unique_ids(raw_boundary_ids)
    if _is_route_a_sheet_interface(route, interface):
        return _route_a_sheet_boundary_volume_ids(
            build_input,
            _entity_by_id(build_input, interface.owner_semantic_ids[0]),
        )
    return _unique_ids(interface.owner_semantic_ids)


def _surface_interface_id(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> str:
    if not _is_route_a_sheet_interface(route, interface):
        return interface.interface_id
    entity = _entity_by_id(build_input, interface.owner_semantic_ids[0])
    boundary_ids = _route_a_sheet_boundary_volume_ids(build_input, entity)
    suffix = interface.interface_id.rsplit("__", 1)[-1]
    return f"MA__{entity.semantic_id}__{'__'.join(boundary_ids)}__{suffix}"


def _is_route_a_sheet_interface(
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> bool:
    return (
        route == "A"
        and interface.metadata.get("recognition_rule")
        == "route_a_surface_sheet_polygon"
    )


def _is_hidden_contact_interface(
    route: RouteLiteral,
    interface: InterfacePlanRecord,
) -> bool:
    return (
        interface.recognition_rule == "coplanar_conductor_contact_patch"
        and route == "B"
    )


def _volume_physical_name(volume: VolumePlanRecord) -> str:
    override = volume.metadata.get("physical_name")
    if isinstance(override, str) and override:
        return override
    physical_owner_id = volume.metadata.get("physical_owner_semantic_id")
    if isinstance(physical_owner_id, str) and physical_owner_id:
        return physical_owner_id
    return volume.owner_semantic_id


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
    if len(kinds) > 2:
        raise ValueError(
            f"{surface.surface_id} has too many interface kinds: {kinds!r}"
        )
    kind_prefix = "_".join(kinds)
    exposed_role = surface.metadata.get("exposed_surface_role")
    boundary_role = surface.metadata.get("boundary_role")
    owner_ids = _surface_owner_ids(surface)
    physical_owner_ids = _surface_physical_owner_ids(surface)
    if physical_owner_ids != owner_ids:
        return _grouped_surface_physical_name(
            surface,
            kind_prefix=kind_prefix,
            physical_owner_ids=physical_owner_ids,
        )
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


def _grouped_surface_physical_name(
    surface: SurfacePlanRecord,
    *,
    kind_prefix: str,
    physical_owner_ids: tuple[str, ...],
) -> str:
    exposed_role = surface.metadata.get("exposed_surface_role")
    suffix = _surface_role_suffix(exposed_role)
    if surface.interface_id is not None:
        parts = surface.interface_id.split("__")
        if len(parts) >= 2 and parts[1] == "CONTACT":
            return "__".join((kind_prefix or parts[0], "CONTACT", *physical_owner_ids))
        if suffix is None and parts:
            suffix = _surface_role_suffix(parts[-1])
        return "__".join(
            (
                kind_prefix or (parts[0] if parts else ""),
                *physical_owner_ids,
                *((suffix,) if suffix else ()),
            )
        )
    return "__".join(
        (
            kind_prefix,
            *physical_owner_ids,
            *((suffix,) if suffix else ()),
        )
    )


def _surface_role_suffix(value: Any) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    if value.startswith("sidewall_") or value.startswith("SIDEWALL_"):
        return "SIDEWALL"
    if value.lower() in {"top", "bottom", "sidewall"}:
        return value.upper()
    if value.upper() in {"TOP", "BOTTOM", "SIDEWALL"}:
        return value.upper()
    return None


def _surface_physical_owner_ids(surface: SurfacePlanRecord) -> tuple[str, ...]:
    owner_ids = surface.metadata.get("physical_owner_semantic_ids")
    if isinstance(owner_ids, str):
        return (owner_ids,)
    if isinstance(owner_ids, Sequence):
        return tuple(str(owner_id) for owner_id in owner_ids)
    return _surface_owner_ids(surface)


def _physical_group_owner_ids(
    build_input: GeometryBuildInput,
    owner_ids: Sequence[str],
) -> tuple[str, ...]:
    entities_by_id = {entity.semantic_id: entity for entity in build_input.entities}
    return _unique_ids(
        _entity_physical_group_id(entities_by_id[owner_id])
        if owner_id in entities_by_id
        else owner_id
        for owner_id in owner_ids
    )


def _entity_physical_group_id(entity: SemanticEntitySpec) -> str:
    if entity.part_role not in HIGH_COUNT_LOCAL_CONDUCTOR_PART_ROLES:
        return entity.semantic_id
    for key in ("physical_group_name", "physical_group_id", "semantic_group_id"):
        value = entity.metadata.get(key)
        if isinstance(value, str) and value:
            return value
    return entity.semantic_id


def _plan_substrate_air_surfaces(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
) -> tuple[SurfacePlanRecord, ...]:
    records: list[SurfacePlanRecord] = list(
        _plan_solution_domain_boundary_surfaces(build_input, route=route)
    )
    surface_index = 0
    for lower, upper, z_um, bounds in _solution_interface_planes(build_input):
        kind = _solution_interface_kind(lower, upper)
        owner_ids = _solution_interface_owner_ids(kind, lower, upper)
        base_loop = _domain_bounds_loop(bounds)
        plane_conductors = _conductor_entities_on_solution_plane(
            build_input,
            route=route,
            lower=lower,
            upper=upper,
            z_um=z_um,
            base_loop=base_loop,
        )
        solution_geometry_refs = _solution_interface_geometry_refs(
            {
                "plane": {"axis": "z", "value_um": z_um},
                "outer_loop": base_loop,
                "hole_loops": (),
            },
            plane_conductors,
        )
        for geometry_ref in solution_geometry_refs:
            interface_id = (
                f"{kind}__{owner_ids[0]}__{owner_ids[1]}__{surface_index:04d}"
            )
            records.append(
                SurfacePlanRecord(
                    surface_id=f"SURF__{interface_id}",
                    owner_semantic_id=owner_ids[0],
                    surface_role="solution_interface",
                    geometry_ref=geometry_ref,
                    interface_id=interface_id,
                    valid_routes=(route,),
                    solver_use="solver_active",
                    metadata={
                        "interface_kinds": (kind,),
                        "owner_semantic_ids": owner_ids,
                        "boundary_volume_ids": (lower.semantic_id, upper.semantic_id),
                    },
                )
            )
            surface_index += 1
    return tuple(records)


def _solution_interface_geometry_refs(
    parent_geometry_ref: Mapping[str, Any],
    plane_conductors: Sequence[SemanticEntitySpec],
) -> tuple[dict[str, Any], ...]:
    """Create live solution-interface patches after removing conductors."""
    import gdstk

    hole_loops = _simple_interior_hole_loops(parent_geometry_ref, plane_conductors)
    if hole_loops is not None:
        return ({**dict(parent_geometry_ref), "hole_loops": hole_loops},)

    base_region = _gdstk_surface_region(parent_geometry_ref)
    conductor_region = tuple(
        polygon
        for entity in plane_conductors
        for polygon in _entity_occupied_region(gdstk, entity)
    )
    live_region = _boolean_gdstk_region(
        gdstk,
        base_region,
        conductor_region,
        "not",
    )
    return _geometry_refs_from_gdstk_region(parent_geometry_ref, live_region)


def _simple_interior_hole_loops(
    parent_geometry_ref: Mapping[str, Any],
    plane_conductors: Sequence[SemanticEntitySpec],
) -> tuple[tuple[tuple[float, float], ...], ...] | None:
    if any(entity.geometry.get("hole_loops") for entity in plane_conductors):
        return None
    base_loop = _clean_loop(parent_geometry_ref["outer_loop"])
    hole_loops = tuple(
        _clean_loop(entity.geometry["outer_loop"])
        for entity in plane_conductors
        if "outer_loop" in entity.geometry
    )
    if len(hole_loops) != len(plane_conductors):
        return None
    if any(not _loop_inside_loop(hole_loop, base_loop) for hole_loop in hole_loops):
        return None
    all_loops = (base_loop, *hole_loops)
    for index, left in enumerate(all_loops):
        for right in all_loops[index + 1 :]:
            if _loops_share_edge_overlap(left, right):
                return None
    return hole_loops


def _loops_share_edge_overlap(
    left: tuple[tuple[float, float], ...],
    right: tuple[tuple[float, float], ...],
) -> bool:
    return any(
        _segment_overlap_interval(left_start, left_end, right_start, right_end)
        is not None
        for left_start, left_end in _ring_edges(left)
        for right_start, right_end in _ring_edges(right)
    )


def _entity_occupied_region(gdstk: Any, entity: SemanticEntitySpec) -> tuple[Any, ...]:
    if "outer_loop" not in entity.geometry:
        return ()
    outer = gdstk.Polygon(_clean_loop(entity.geometry["outer_loop"]))
    holes = tuple(
        gdstk.Polygon(_clean_loop(hole_loop))
        for hole_loop in entity.geometry.get("hole_loops", ())
    )
    if not holes:
        return _filter_gdstk_polygons((outer,))
    return _boolean_gdstk_region(gdstk, (outer,), holes, "not")


def _solution_domain_sidewall_geometry_refs(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    solution: SemanticEntitySpec,
    outer_loop: tuple[tuple[float, float], ...],
    z_min_um: float,
    z_max_um: float,
) -> tuple[dict[str, Any], ...]:
    refs: list[dict[str, Any]] = []
    for edge_index, (start, end) in enumerate(_ring_edges(_clean_loop(outer_loop))):
        edge_z_min_um = _solution_boundary_edge_z_min_um(
            build_input,
            route=route,
            solution=solution,
            start=start,
            end=end,
            default_z_min_um=z_min_um,
        )
        edge_z_max_um = _solution_boundary_edge_z_max_um(
            build_input,
            route=route,
            solution=solution,
            start=start,
            end=end,
            default_z_max_um=z_max_um,
        )
        if edge_z_max_um - edge_z_min_um <= _INSET_EPS_UM:
            continue
        refs.append(
            {
                "quad_points": (
                    (start[0], start[1], edge_z_min_um),
                    (end[0], end[1], edge_z_min_um),
                    (end[0], end[1], edge_z_max_um),
                    (start[0], start[1], edge_z_max_um),
                ),
                "sidewall_ring_role": "outer",
                "sidewall_edge_index": edge_index,
            }
        )
    return tuple(refs)


def _solution_boundary_edge_z_min_um(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    solution: SemanticEntitySpec,
    start: tuple[float, float],
    end: tuple[float, float],
    default_z_min_um: float,
) -> float:
    if not _is_air_like_solution_entity(solution):
        return default_z_min_um
    z_min_um = default_z_min_um
    for entity in build_input.entities:
        if (
            _is_solution_entity(entity)
            or entity.route_representations.get(route) not in {
                "cutout_boundary_shell",
                "material_volume",
            }
            or "outer_loop" not in entity.geometry
        ):
            continue
        entity_z_min_um, entity_z_max_um = _entity_z_range_um(entity)
        if not _same_z(entity_z_min_um, default_z_min_um):
            continue
        if _edge_matches_loop_edge(start, end, entity.geometry["outer_loop"]):
            z_min_um = max(z_min_um, entity_z_max_um)
    return z_min_um


def _solution_boundary_edge_z_max_um(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    solution: SemanticEntitySpec,
    start: tuple[float, float],
    end: tuple[float, float],
    default_z_max_um: float,
) -> float:
    if not _is_air_like_solution_entity(solution):
        return default_z_max_um
    z_max_um = default_z_max_um
    for entity in build_input.entities:
        if (
            _is_solution_entity(entity)
            or entity.route_representations.get(route) not in {
                "cutout_boundary_shell",
                "material_volume",
            }
            or "outer_loop" not in entity.geometry
        ):
            continue
        entity_z_min_um, entity_z_max_um = _entity_z_range_um(entity)
        if not _same_z(entity_z_max_um, default_z_max_um):
            continue
        if _edge_matches_loop_edge(start, end, entity.geometry["outer_loop"]):
            z_max_um = min(z_max_um, entity_z_min_um)
    return z_max_um


def _edge_matches_loop_edge(
    start: tuple[float, float],
    end: tuple[float, float],
    loop: Any,
) -> bool:
    edge_key = {_point2d_key(start), _point2d_key(end)}
    return any(
        {_point2d_key(edge_start), _point2d_key(edge_end)} == edge_key
        for edge_start, edge_end in _ring_edges(_clean_loop(loop))
    )


def _point2d_key(point: tuple[float, float]) -> tuple[float, float]:
    return (round(float(point[0]), 9), round(float(point[1]), 9))


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
        for boundary_role in ("bottom", "top"):
            geometry_ref = _solution_exterior_face_geometry_ref(
                build_input,
                entity,
                boundary_role,
            )
            if geometry_ref is None:
                continue
            records.append(
                SurfacePlanRecord(
                    surface_id=(
                        f"SURF__BOUNDARY__{entity.semantic_id}__"
                        f"{boundary_role.upper()}"
                    ),
                    owner_semantic_id=entity.semantic_id,
                    surface_role="domain_boundary",
                    geometry_ref=geometry_ref,
                    valid_routes=(route,),
                    metadata={
                        "owner_semantic_ids": (entity.semantic_id,),
                        "boundary_volume_ids": (entity.semantic_id,),
                        "boundary_role": boundary_role,
                    },
                )
            )
        for edge_index, geometry_ref in enumerate(
            _solution_domain_sidewall_geometry_refs(
                build_input,
                route=route,
                solution=entity,
                outer_loop=loop,
                z_min_um=z_min_um,
                z_max_um=z_max_um,
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
                        "boundary_volume_ids": (entity.semantic_id,),
                        "boundary_role": "sidewall",
                    },
                )
            )
    return tuple(records)


def _cutout_shell_surface_ids(
    build_input: GeometryBuildInput,
    route: RouteLiteral,
    entity: SemanticEntitySpec,
    *,
    interfaces: tuple[InterfacePlanRecord, ...] = (),
    contact_faces: Mapping[
        tuple[str, str],
        tuple[tuple[tuple[float, float], ...], ...],
    ] | None = None,
) -> tuple[str, ...]:
    contact_faces = contact_faces or {}
    top_bottom_ids: list[str] = []
    for shell_part in ("top", "bottom"):
        base_surface_id = _conductor_boundary_surface_id(
            route,
            entity,
            "cutout_boundary_shell",
            shell_part,
        )
        base_geometry_ref = {
            **_route_entity_geometry_ref(
                build_input,
                route,
                entity,
                representation="cutout_boundary_shell",
                interfaces=interfaces,
            ),
            "shell_part": shell_part,
        }
        face_geometry_refs = _subtract_contact_patches_from_face(
            base_geometry_ref,
            contact_faces.get((entity.semantic_id, shell_part), ()),
        )
        if len(face_geometry_refs) == 1:
            top_bottom_ids.append(base_surface_id)
        else:
            top_bottom_ids.extend(
                f"{base_surface_id}__P{index:04d}"
                for index, _ in enumerate(face_geometry_refs)
            )
    sidewall_ids = tuple(
        _conductor_boundary_surface_id(
            route,
            entity,
            "cutout_boundary_shell",
            f"sidewall_{edge_index:04d}",
        )
        for edge_index, _ in enumerate(
            _conductor_sidewall_geometry_refs(
                build_input,
                route=route,
                entity=entity,
                representation="cutout_boundary_shell",
                adjacent_solution_id=_conductor_sidewall_adjacent_solution_id(
                    build_input,
                    entity,
                ),
                interfaces=interfaces,
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


def _conductor_face_interface_id(
    interface_kind: str,
    semantic_id: str,
    adjacent_id: str,
    shell_part: str,
    face_index: int | None,
) -> str:
    suffix = "" if face_index is None else f"__P{face_index:04d}"
    return (
        f"{interface_kind}__{semantic_id}__{adjacent_id}__"
        f"{shell_part.upper()}{suffix}"
    )


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
    import gdstk

    clean_loop = _clean_loop(loop)
    clean_container = _clean_loop(container)
    loop_area = abs(_polygon_area(clean_loop))
    intersection = _boolean_gdstk_region(
        gdstk,
        (gdstk.Polygon(clean_loop),),
        (gdstk.Polygon(clean_container),),
        "and",
    )
    intersection_area = sum(
        abs(_polygon_area(polygon.points)) for polygon in intersection
    )
    return loop_area - intersection_area <= max(_INSET_EPS_UM, loop_area * 1e-9)


def _loop_centroid(loop: Any) -> tuple[float, float]:
    points = tuple((float(point[0]), float(point[1])) for point in loop)
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _surface_boundary_volume_ids(
    surface: SurfacePlanRecord,
    *,
    known_entity_ids: set[str],
) -> tuple[str, ...]:
    raw = surface.metadata.get(
        "boundary_volume_ids",
        surface.metadata.get("owner_semantic_ids", (surface.owner_semantic_id,)),
    )
    if isinstance(raw, str):
        values = (raw,)
    else:
        values = tuple(str(value) for value in raw)
    result = _unique_ids(value for value in values if value in known_entity_ids)
    if len(result) > 2:
        raise ValueError(
            f"{surface.surface_id} belongs to more than two volumes: {result!r}"
        )
    return result


def _surface_orientation_for_volume(
    surface: SurfacePlanRecord,
    entity: SemanticEntitySpec,
) -> SurfaceOrientationLiteral:
    """Orient a planned surface as an outward face of one owning volume."""
    normal = _surface_normal_vector(surface)
    surface_centroid = _surface_centroid(surface)
    volume_center = _entity_volume_center_um(entity)
    outward = tuple(
        surface_centroid[index] - volume_center[index]
        for index in range(3)
    )
    dot = sum(normal[index] * outward[index] for index in range(3))
    if abs(dot) <= 1e-9:
        raise ValueError(
            f"{surface.surface_id} has ambiguous orientation for "
            f"{entity.semantic_id}"
        )
    return "forward" if dot > 0 else "reversed"


def _surface_normal_vector(surface: SurfacePlanRecord) -> tuple[float, float, float]:
    if surface.normal_hint is not None:
        return tuple(float(value) for value in surface.normal_hint)
    ring = _surface_ring3d_specs(surface)[0][2]
    origin = ring[0]
    for first_index in range(1, len(ring) - 1):
        first = _vector_subtract(ring[first_index], origin)
        for second_index in range(first_index + 1, len(ring)):
            second = _vector_subtract(ring[second_index], origin)
            normal = _vector_cross(first, second)
            length_sq = sum(value * value for value in normal)
            if length_sq > 1e-18:
                return normal
    raise ValueError(f"{surface.surface_id} has no nondegenerate normal")


def _surface_centroid(surface: SurfacePlanRecord) -> tuple[float, float, float]:
    points = tuple(
        coordinate
        for _, _, ring in _surface_ring3d_specs(surface)
        for coordinate in ring
    )
    if not points:
        raise ValueError(f"{surface.surface_id} has no coordinates")
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
        sum(point[2] for point in points) / len(points),
    )


def _entity_volume_center_um(
    entity: SemanticEntitySpec,
) -> tuple[float, float, float]:
    if _is_solution_entity(entity):
        bounds = _solution_bounds(entity)
        return (
            (float(bounds["x_min_um"]) + float(bounds["x_max_um"])) / 2.0,
            (float(bounds["y_min_um"]) + float(bounds["y_max_um"])) / 2.0,
            (float(entity.geometry["z_min_um"]) + float(entity.geometry["z_max_um"]))
            / 2.0,
        )
    if "outer_loop" not in entity.geometry:
        raise ValueError(f"{entity.semantic_id} requires outer_loop for volume center")
    loop = _clean_loop(entity.geometry["outer_loop"])
    z_min_um, z_max_um = _entity_z_range_um(entity)
    return (
        (min(point[0] for point in loop) + max(point[0] for point in loop)) / 2.0,
        (min(point[1] for point in loop) + max(point[1] for point in loop)) / 2.0,
        (z_min_um + z_max_um) / 2.0,
    )


def _vector_subtract(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def _vector_cross(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _unique_ids(values: Any) -> tuple[str, ...]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        item = str(value)
        if item in seen:
            continue
        result.append(item)
        seen.add(item)
    return tuple(result)


def _inset_family_ids(surface: SurfacePlanRecord) -> tuple[str, ...]:
    """Return coplanar inset family ids for reviewable joint partitioning.

    A family is keyed by plane plus boundary volume. `SA` and `MS` children on a
    substrate top face therefore share the substrate family even though their
    other owner differs. Sidewall surfaces do not get this planar family id.
    """
    if "outer_loop" not in surface.geometry_ref:
        return ()
    plane = surface.geometry_ref.get("plane") or surface.geometry_ref.get(
        "contact_plane",
    )
    if not isinstance(plane, Mapping):
        return ()
    axis = str(plane.get("axis", "")).lower()
    if not axis:
        return ()
    value_um = round(
        _geometry_ref_surface_z_um(surface.geometry_ref)
        if axis == "z"
        else float(plane["value_um"]),
        9,
    )
    volumes = surface.metadata.get("boundary_volume_ids", ())
    if isinstance(volumes, str):
        volume_ids = (volumes,)
    else:
        volume_ids = tuple(str(value) for value in volumes)
    return tuple(
        f"COPLANAR_INSET__{axis.upper()}_{value_um}__VOL_{volume_id}"
        for volume_id in sorted(set(volume_ids))
    )


def _parse_inset_family_id(family_id: str) -> tuple[str, str]:
    prefix = "COPLANAR_INSET__"
    if not family_id.startswith(prefix) or "__VOL_" not in family_id:
        raise ValueError(f"invalid coplanar inset family id: {family_id}")
    plane_key, boundary_volume_id = family_id.removeprefix(prefix).split(
        "__VOL_",
        1,
    )
    if not plane_key or not boundary_volume_id:
        raise ValueError(f"invalid coplanar inset family id: {family_id}")
    return plane_key, boundary_volume_id


def _ordered_loop_signature(
    curve_refs: tuple[CurveRefRecord, ...],
) -> tuple[tuple[str, int], ...]:
    """Return a rotation-stable loop signature without discarding orientation."""
    signature = tuple((ref.curve_id, ref.orientation) for ref in curve_refs)
    if not signature:
        return ()
    rotations = (
        signature[index:] + signature[:index]
        for index in range(len(signature))
    )
    return min(rotations)


def _surface_planar_bounds(surface: SurfacePlanRecord) -> dict[str, float]:
    points = [
        point
        for loop in (
            surface.geometry_ref["outer_loop"],
            *surface.geometry_ref.get("hole_loops", ()),
        )
        for point in _clean_loop(loop)
    ]
    return {
        "x_min_um": min(point[0] for point in points),
        "y_min_um": min(point[1] for point in points),
        "x_max_um": max(point[0] for point in points),
        "y_max_um": max(point[1] for point in points),
    }


def _merge_bounds(
    first: Mapping[str, float],
    second: Mapping[str, float],
) -> dict[str, float]:
    return {
        "x_min_um": min(float(first["x_min_um"]), float(second["x_min_um"])),
        "y_min_um": min(float(first["y_min_um"]), float(second["y_min_um"])),
        "x_max_um": max(float(first["x_max_um"]), float(second["x_max_um"])),
        "y_max_um": max(float(first["y_max_um"]), float(second["y_max_um"])),
    }


def _bounds_touch_or_overlap(
    first: Mapping[str, float],
    second: Mapping[str, float],
) -> bool:
    return not (
        float(first["x_max_um"]) < float(second["x_min_um"]) - _INSET_EPS_UM
        or float(second["x_max_um"]) < float(first["x_min_um"]) - _INSET_EPS_UM
        or float(first["y_max_um"]) < float(second["y_min_um"]) - _INSET_EPS_UM
        or float(second["y_max_um"]) < float(first["y_min_um"]) - _INSET_EPS_UM
    )


def _route_a_sheet_boundary_volume_ids(
    build_input: GeometryBuildInput,
    entity: SemanticEntitySpec,
) -> tuple[str, ...]:
    return _unique_ids(
        (
            _conductor_face_adjacent_solution_id(build_input, entity, "bottom"),
            _conductor_face_adjacent_solution_id(build_input, entity, "top"),
        )
    )


def _route_a_sheet_plane_z_um(
    build_input: GeometryBuildInput,
    entity: SemanticEntitySpec,
) -> float:
    z_min_um, z_max_um = _entity_z_range_um(entity)
    for boundary_id, z_um in (
        (
            _conductor_face_adjacent_solution_id(build_input, entity, "bottom"),
            z_min_um,
        ),
        (
            _conductor_face_adjacent_solution_id(build_input, entity, "top"),
            z_max_um,
        ),
    ):
        if not _is_air_like_solution_entity(_entity_by_id(build_input, boundary_id)):
            return z_um
    return z_min_um


def _conductor_boundary_volume_ids(
    route: RouteLiteral,
    entity: SemanticEntitySpec,
    adjacent_solution_id: str,
) -> tuple[str, ...]:
    if route == "C":
        return (entity.semantic_id, adjacent_solution_id)
    return (adjacent_solution_id,)


def _conductor_face_adjacent_solution_id(
    build_input: GeometryBuildInput,
    entity: SemanticEntitySpec,
    face: str,
) -> str:
    z_min_um, z_max_um = _entity_z_range_um(entity)
    face_z_um = z_min_um if face == "bottom" else z_max_um
    point = _loop_centroid(entity.geometry["outer_loop"])
    exact_candidates = [
        solution
        for solution in _solution_entities(build_input)
        if _bounds_contains_point(_solution_bounds(solution), point)
        and (
            _same_z(float(solution.geometry["z_max_um"]), face_z_um)
            if face == "bottom"
            else _same_z(float(solution.geometry["z_min_um"]), face_z_um)
        )
    ]
    if len(exact_candidates) == 1:
        return exact_candidates[0].semantic_id
    if len(exact_candidates) > 1:
        raise ValueError(
            f"{entity.semantic_id} {face} face touches multiple solution volumes"
        )

    containing_candidates = [
        solution
        for solution in _solution_entities(build_input)
        if _bounds_contains_point(_solution_bounds(solution), point)
        and float(solution.geometry["z_min_um"]) < face_z_um
        and face_z_um < float(solution.geometry["z_max_um"])
    ]
    if len(containing_candidates) == 1:
        return containing_candidates[0].semantic_id
    if len(containing_candidates) > 1:
        raise ValueError(
            f"{entity.semantic_id} {face} face is inside multiple solution volumes"
        )
    raise ValueError(f"{entity.semantic_id} {face} face has no adjacent solution")


def _conductor_sidewall_adjacent_solution_id(
    build_input: GeometryBuildInput,
    entity: SemanticEntitySpec,
) -> str:
    z_min_um, z_max_um = _entity_z_range_um(entity)
    point = _loop_centroid(entity.geometry["outer_loop"])
    mid_z_um = (z_min_um + z_max_um) / 2.0
    candidates = [
        solution
        for solution in _solution_entities(build_input)
        if _bounds_contains_point(_solution_bounds(solution), point)
        and float(solution.geometry["z_min_um"]) < mid_z_um
        and mid_z_um < float(solution.geometry["z_max_um"])
    ]
    if len(candidates) == 1:
        return candidates[0].semantic_id
    if len(candidates) > 1:
        raise ValueError(f"{entity.semantic_id} sidewall is inside multiple volumes")
    raise ValueError(f"{entity.semantic_id} sidewall has no adjacent solution")


def _conductor_solution_interface_kind(solution: SemanticEntitySpec) -> str:
    return "MA" if _is_air_like_solution_entity(solution) else "MS"


def _solution_interface_planes(
    build_input: GeometryBuildInput,
) -> tuple[
    tuple[SemanticEntitySpec, SemanticEntitySpec, float, Mapping[str, float]],
    ...,
]:
    records: list[
        tuple[SemanticEntitySpec, SemanticEntitySpec, float, Mapping[str, float]]
    ] = []
    solutions = _solution_entities(build_input)
    for index, first in enumerate(solutions):
        for second in solutions[index + 1 :]:
            first_bounds = _solution_bounds(first)
            second_bounds = _solution_bounds(second)
            overlap = _intersect_bounds(first_bounds, second_bounds)
            if overlap is None:
                continue
            if _same_z(
                float(first.geometry["z_max_um"]),
                float(second.geometry["z_min_um"]),
            ):
                records.append(
                    (first, second, float(first.geometry["z_max_um"]), overlap)
                )
            elif _same_z(
                float(second.geometry["z_max_um"]),
                float(first.geometry["z_min_um"]),
            ):
                records.append(
                    (second, first, float(second.geometry["z_max_um"]), overlap)
                )
    return tuple(records)


def _conductor_entities_on_solution_plane(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    lower: SemanticEntitySpec,
    upper: SemanticEntitySpec,
    z_um: float,
    base_loop: tuple[tuple[float, float], ...],
) -> tuple[SemanticEntitySpec, ...]:
    pair_ids = {lower.semantic_id, upper.semantic_id}
    records: list[SemanticEntitySpec] = []
    for entity in build_input.entities:
        if (
            _is_solution_entity(entity)
            or entity.route_representations.get(route) is None
            or "outer_loop" not in entity.geometry
            or not _loop_inside_loop(entity.geometry["outer_loop"], base_loop)
        ):
            continue
        representation = entity.route_representations.get(route)
        if representation == "surface_sheet":
            if (
                set(_route_a_sheet_boundary_volume_ids(build_input, entity)) == pair_ids
                and _same_z(_route_a_sheet_plane_z_um(build_input, entity), z_um)
            ):
                records.append(entity)
            continue
        if any(
            _same_z(face_z, z_um)
            and _conductor_face_adjacent_solution_id(
                build_input,
                entity,
                face,
            )
            in pair_ids
            for face, face_z in zip(
                ("bottom", "top"),
                _entity_z_range_um(entity),
                strict=True,
            )
        ):
            records.append(entity)
    return tuple(records)


def _solution_interface_kind(
    lower: SemanticEntitySpec,
    upper: SemanticEntitySpec,
) -> str:
    if _is_air_like_solution_entity(lower) or _is_air_like_solution_entity(upper):
        return "AA" if (
            _is_air_like_solution_entity(lower)
            and _is_air_like_solution_entity(upper)
        ) else "SA"
    return "SS"


def _solution_interface_owner_ids(
    kind: str,
    lower: SemanticEntitySpec,
    upper: SemanticEntitySpec,
) -> tuple[str, str]:
    if kind == "SA":
        if _is_air_like_solution_entity(lower):
            return (upper.semantic_id, lower.semantic_id)
        return (lower.semantic_id, upper.semantic_id)
    return (lower.semantic_id, upper.semantic_id)


def _solution_exterior_face_geometry_ref(
    build_input: GeometryBuildInput,
    entity: SemanticEntitySpec,
    face: str,
) -> dict[str, Any] | None:
    z_key = "z_max_um" if face == "top" else "z_min_um"
    z_um = float(entity.geometry[z_key])
    bounds = _solution_bounds(entity)
    holes: list[tuple[tuple[float, float], ...]] = []
    for other in _solution_entities(build_input):
        if other.semantic_id == entity.semantic_id:
            continue
        other_bounds = _solution_bounds(other)
        overlap = _intersect_bounds(bounds, other_bounds)
        if overlap is None:
            continue
        if face == "top" and _same_z(float(other.geometry["z_min_um"]), z_um):
            if _same_bounds(bounds, overlap):
                return None
            holes.append(_domain_bounds_loop(overlap))
        if face == "bottom" and _same_z(float(other.geometry["z_max_um"]), z_um):
            if _same_bounds(bounds, overlap):
                return None
            holes.append(_domain_bounds_loop(overlap))
    return {
        "plane": {"axis": "z", "value_um": z_um},
        "outer_loop": _domain_bounds_loop(bounds),
        "hole_loops": tuple(holes),
    }


def _solution_entities(
    build_input: GeometryBuildInput,
) -> tuple[SemanticEntitySpec, ...]:
    cache_key = id(build_input)
    cached = _SOLUTION_ENTITIES_BY_INPUT_ID.get(cache_key)
    if cached is not None and cached[0] is build_input:
        return cached[1]
    # ponytail: per-input cache; replace with explicit context if planner state grows.
    records = tuple(
        entity for entity in build_input.entities if _is_solution_entity(entity)
    )
    _SOLUTION_ENTITIES_BY_INPUT_ID[cache_key] = (build_input, records)
    return records


def _solution_bounds(entity: SemanticEntitySpec) -> Mapping[str, float]:
    bounds = entity.geometry.get("domain_bounds_um")
    if not isinstance(bounds, Mapping):
        raise ValueError(f"{entity.semantic_id} requires domain_bounds_um")
    return bounds


def _intersect_bounds(
    first: Mapping[str, Any],
    second: Mapping[str, Any],
) -> dict[str, float] | None:
    bounds = {
        "x_min_um": max(float(first["x_min_um"]), float(second["x_min_um"])),
        "y_min_um": max(float(first["y_min_um"]), float(second["y_min_um"])),
        "x_max_um": min(float(first["x_max_um"]), float(second["x_max_um"])),
        "y_max_um": min(float(first["y_max_um"]), float(second["y_max_um"])),
    }
    if (
        bounds["x_min_um"] >= bounds["x_max_um"]
        or bounds["y_min_um"] >= bounds["y_max_um"]
    ):
        return None
    return bounds


def _same_bounds(first: Mapping[str, Any], second: Mapping[str, Any]) -> bool:
    return all(
        abs(float(first[key]) - float(second[key])) <= _INSET_EPS_UM
        for key in ("x_min_um", "y_min_um", "x_max_um", "y_max_um")
    )


def _bounds_contains_point(
    bounds: Mapping[str, Any],
    point: tuple[float, float],
) -> bool:
    x, y = point
    return (
        float(bounds["x_min_um"]) <= x <= float(bounds["x_max_um"])
        and float(bounds["y_min_um"]) <= y <= float(bounds["y_max_um"])
    )


def _entity_z_range_um(entity: SemanticEntitySpec) -> tuple[float, float]:
    z_min_um = float(entity.geometry.get("z_min_um", entity.geometry.get("z_um", 0.0)))
    return z_min_um, z_min_um + float(entity.geometry.get("thickness_um", 0.0))


def _same_z(left: float, right: float) -> bool:
    return abs(left - right) <= _INSET_EPS_UM


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


def _route_entity_geometry_ref(
    build_input: GeometryBuildInput,
    route: RouteLiteral,
    entity: SemanticEntitySpec,
    *,
    representation: str,
    interfaces: tuple[InterfacePlanRecord, ...] = (),
) -> dict[str, Any]:
    del build_input
    geometry_ref = _entity_geometry_ref(entity, representation=representation)
    if route != "A" or representation != "cutout_boundary_shell":
        return geometry_ref

    z_min_um, z_max_um = _entity_z_range_um(entity)
    for interface in interfaces:
        if interface.recognition_rule != "coplanar_conductor_contact_patch":
            continue
        metadata = interface.metadata
        plane = metadata.get("contact_plane") or metadata.get("plane")
        if not isinstance(plane, Mapping) or plane.get("axis") != "z":
            continue
        plane_z_um = float(plane["value_um"])
        if (
            metadata.get("upper_entity_id") == entity.semantic_id
            and metadata.get("upper_face") == "bottom"
        ):
            z_min_um = min(z_min_um, plane_z_um)
        if (
            metadata.get("lower_entity_id") == entity.semantic_id
            and metadata.get("lower_face") == "top"
        ):
            z_max_um = max(z_max_um, plane_z_um)

    if z_max_um <= z_min_um:
        raise ValueError(f"{entity.semantic_id} Route A cutout body has empty z range")
    if not _same_z(z_min_um, _entity_z_range_um(entity)[0]) or not _same_z(
        z_max_um,
        _entity_z_range_um(entity)[1],
    ):
        geometry_ref["z_um"] = z_min_um
        geometry_ref["z_min_um"] = z_min_um
        geometry_ref["thickness_um"] = z_max_um - z_min_um
        geometry_ref["route_a_cutout_z_range_um"] = (z_min_um, z_max_um)
    return geometry_ref


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


def _conductor_sidewall_geometry_refs(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    entity: SemanticEntitySpec,
    representation: str,
    adjacent_solution_id: str,
    interfaces: tuple[InterfacePlanRecord, ...] = (),
) -> tuple[dict[str, Any], ...]:
    refs = _sidewall_geometry_refs(
        _route_entity_geometry_ref(
            build_input,
            route,
            entity,
            representation=representation,
            interfaces=interfaces,
        )
    )
    edge_index = _conductor_boundary_edge_index(
        build_input,
        route=route,
        entity=entity,
    )
    if route == "C":
        return _route_c_conductor_sidewall_geometry_refs(
            entity=entity,
            adjacent_solution=_entity_by_id(build_input, adjacent_solution_id),
            refs=refs,
            edge_index=edge_index,
        )
    adjacent_solution = _entity_by_id(build_input, adjacent_solution_id)
    exposed_refs: list[dict[str, Any]] = []
    for geometry_ref in refs:
        if _sidewall_on_solution_outer_boundary(geometry_ref, adjacent_solution):
            continue
        exposed_refs.extend(
            _trim_sidewall_ref_against_route_conductors(
                geometry_ref=geometry_ref,
                edge_index=edge_index,
            )
        )
    return tuple(exposed_refs)


def _route_c_conductor_sidewall_geometry_refs(
    *,
    entity: SemanticEntitySpec,
    adjacent_solution: SemanticEntitySpec,
    refs: tuple[dict[str, Any], ...],
    edge_index: Mapping[
        tuple[tuple[float, float], float],
        Sequence[tuple[tuple[float, float], tuple[float, float], str]],
    ],
) -> tuple[dict[str, Any], ...]:
    result: list[dict[str, Any]] = []
    for geometry_ref in refs:
        points = geometry_ref.get("quad_points", ())
        if len(points) != 4:
            result.append(dict(geometry_ref))
            continue
        start = (float(points[0][0]), float(points[0][1]))
        end = (float(points[1][0]), float(points[1][1]))
        exterior_intervals = (
            ((0.0, 1.0),)
            if _sidewall_on_solution_outer_boundary(geometry_ref, adjacent_solution)
            else ()
        )
        contact_intervals = _route_c_contact_intervals(
            entity=entity,
            start=start,
            end=end,
            edge_index=edge_index,
        )
        result.extend(
            _sidewall_subsegment_geometry_ref(
                geometry_ref,
                interval_start,
                interval_end,
                extra={"solution_exterior_boundary": True},
            )
            for interval_start, interval_end in exterior_intervals
        )
        result.extend(
            _sidewall_subsegment_geometry_ref(
                geometry_ref,
                interval_start,
                interval_end,
                extra={"adjacent_conductor_semantic_id": adjacent_id},
            )
            for (
                interval_start,
                interval_end,
                adjacent_id,
                create_surface,
            ) in contact_intervals
            if create_surface
        )
        blocked = (
            *exterior_intervals,
            *((start, end) for start, end, _, _ in contact_intervals),
        )
        result.extend(
            _sidewall_subsegment_geometry_ref(
                geometry_ref,
                interval_start,
                interval_end,
            )
            for interval_start, interval_end in _interval_complement(blocked)
        )
    return tuple(result)


def _conductor_boundary_edge_index(
    build_input: GeometryBuildInput,
    *,
    route: RouteLiteral,
    entity: SemanticEntitySpec,
) -> dict[
    tuple[tuple[float, float], float],
    tuple[tuple[tuple[float, float], tuple[float, float], str], ...],
]:
    if entity.part_role in HIGH_COUNT_LOCAL_CONDUCTOR_PART_ROLES:
        return {}
    index: dict[
        tuple[tuple[float, float], float],
        list[tuple[tuple[float, float], tuple[float, float], str]],
    ] = {}
    for other in build_input.entities:
        if not _can_trim_against_route_conductor(
            route,
            entity=entity,
            other=other,
        ):
            continue
        for other_start, other_end in _entity_boundary_edges(other):
            line_key = _line_key_2d(other_start, other_end)
            if line_key is None:
                continue
            index.setdefault(line_key, []).append(
                (other_start, other_end, other.semantic_id)
            )
    return {
        line_key: tuple(records)
        for line_key, records in index.items()
    }


def _candidate_boundary_edges(
    edge_index: Mapping[
        tuple[tuple[float, float], float],
        Sequence[tuple[tuple[float, float], tuple[float, float], str]],
    ],
    start: tuple[float, float],
    end: tuple[float, float],
) -> Sequence[tuple[tuple[float, float], tuple[float, float], str]]:
    line_key = _line_key_2d(start, end)
    if line_key is None:
        return ()
    return edge_index.get(line_key, ())


def _route_c_contact_intervals(
    entity: SemanticEntitySpec,
    start: tuple[float, float],
    end: tuple[float, float],
    edge_index: Mapping[
        tuple[tuple[float, float], float],
        Sequence[tuple[tuple[float, float], tuple[float, float], str]],
    ],
) -> tuple[tuple[float, float, str, bool], ...]:
    records: list[tuple[float, float, str, bool]] = []
    for other_start, other_end, other_semantic_id in _candidate_boundary_edges(
        edge_index,
        start,
        end,
    ):
        interval = _segment_overlap_interval(start, end, other_start, other_end)
        if interval is None:
            continue
        records.append(
            (
                interval[0],
                interval[1],
                other_semantic_id,
                entity.semantic_id < other_semantic_id,
            )
        )
    return tuple(records)


def _trim_sidewall_ref_against_route_conductors(
    *,
    geometry_ref: Mapping[str, Any],
    edge_index: Mapping[
        tuple[tuple[float, float], float],
        Sequence[tuple[tuple[float, float], tuple[float, float], str]],
    ],
) -> tuple[dict[str, Any], ...]:
    points = geometry_ref.get("quad_points", ())
    if len(points) != 4:
        return (dict(geometry_ref),)
    start = (float(points[0][0]), float(points[0][1]))
    end = (float(points[1][0]), float(points[1][1]))
    covered_intervals: list[tuple[float, float]] = []
    for other_start, other_end, _ in _candidate_boundary_edges(
        edge_index,
        start,
        end,
    ):
        interval = _segment_overlap_interval(start, end, other_start, other_end)
        if interval is None:
            continue
        covered_intervals.append(interval)
    if not covered_intervals:
        return (dict(geometry_ref),)
    return tuple(
        _sidewall_subsegment_geometry_ref(
            geometry_ref,
            start_parameter,
            end_parameter,
        )
        for start_parameter, end_parameter in _interval_complement(
            covered_intervals,
        )
        if end_parameter - start_parameter > _INSET_EPS_UM
    )


def _can_trim_against_route_conductor(
    route: RouteLiteral,
    *,
    entity: SemanticEntitySpec,
    other: SemanticEntitySpec,
) -> bool:
    if (
        other.semantic_id == entity.semantic_id
        or _is_solution_entity(other)
        or other.route_representations.get(route) is None
        or "outer_loop" not in other.geometry
    ):
        return False
    z_min_um, z_max_um = _entity_z_range_um(entity)
    other_z_min_um, other_z_max_um = _entity_z_range_um(other)
    return _same_z(z_min_um, other_z_min_um) and _same_z(z_max_um, other_z_max_um)


def _entity_boundary_edges(
    entity: SemanticEntitySpec,
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    return tuple(
        edge
        for loop in (
            entity.geometry["outer_loop"],
            *entity.geometry.get("hole_loops", ()),
        )
        for edge in _ring_edges(_clean_loop(loop))
    )


def _segment_overlap_interval(
    start: tuple[float, float],
    end: tuple[float, float],
    other_start: tuple[float, float],
    other_end: tuple[float, float],
) -> tuple[float, float] | None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-18:
        return None

    def parameter(point: tuple[float, float]) -> float | None:
        cross = (point[0] - start[0]) * dy - (point[1] - start[1]) * dx
        if abs(cross) > 1e-9:
            return None
        return ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq

    first = parameter(other_start)
    second = parameter(other_end)
    if first is None or second is None:
        return None
    overlap_start = max(0.0, min(first, second))
    overlap_end = min(1.0, max(first, second))
    if overlap_end - overlap_start <= _INSET_EPS_UM:
        return None
    return overlap_start, overlap_end


def _line_key_2d(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[tuple[float, float], float] | None:
    start_key = tuple(round(float(value), 9) for value in start)
    end_key = tuple(round(float(value), 9) for value in end)
    direction = (
        end_key[0] - start_key[0],
        end_key[1] - start_key[1],
    )
    scale = max(abs(value) for value in direction)
    if scale <= 1e-18:
        return None
    unit = tuple(round(value / scale, 9) for value in direction)
    for value in unit:
        if abs(value) <= 1e-18:
            continue
        if value < 0:
            unit = (-unit[0], -unit[1])
        break
    offset = round(start_key[0] * unit[1] - start_key[1] * unit[0], 9)
    return unit, offset


def _interval_complement(
    intervals: Sequence[tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
    merged: list[tuple[float, float]] = []
    for start, end in sorted(intervals):
        start = max(0.0, min(1.0, start))
        end = max(0.0, min(1.0, end))
        if end - start <= _INSET_EPS_UM:
            continue
        if not merged or start > merged[-1][1] + _INSET_EPS_UM:
            merged.append((start, end))
        else:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    exposed: list[tuple[float, float]] = []
    cursor = 0.0
    for start, end in merged:
        if start - cursor > _INSET_EPS_UM:
            exposed.append((cursor, start))
        cursor = max(cursor, end)
    if 1.0 - cursor > _INSET_EPS_UM:
        exposed.append((cursor, 1.0))
    return tuple(exposed)


def _sidewall_subsegment_geometry_ref(
    geometry_ref: Mapping[str, Any],
    start_parameter: float,
    end_parameter: float,
    *,
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    points = tuple(
        (float(point[0]), float(point[1]), float(point[2]))
        for point in geometry_ref["quad_points"]
    )
    return {
        **dict(geometry_ref),
        "quad_points": (
            _interpolate_3d(points[0], points[1], start_parameter),
            _interpolate_3d(points[0], points[1], end_parameter),
            _interpolate_3d(points[3], points[2], end_parameter),
            _interpolate_3d(points[3], points[2], start_parameter),
        ),
        "trimmed_by_conductor_contact": True,
        **dict(extra or {}),
    }


def _interpolate_3d(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    parameter: float,
) -> tuple[float, float, float]:
    return (
        start[0] + (end[0] - start[0]) * parameter,
        start[1] + (end[1] - start[1]) * parameter,
        start[2] + (end[2] - start[2]) * parameter,
    )


def _sidewall_on_solution_outer_boundary(
    geometry_ref: Mapping[str, Any],
    solution: SemanticEntitySpec,
) -> bool:
    points = geometry_ref.get("quad_points", ())
    if len(points) != 4:
        return False
    start = (float(points[0][0]), float(points[0][1]))
    end = (float(points[1][0]), float(points[1][1]))
    bounds = _solution_bounds(solution)
    x_min = float(bounds["x_min_um"])
    x_max = float(bounds["x_max_um"])
    y_min = float(bounds["y_min_um"])
    y_max = float(bounds["y_max_um"])
    if _same_z(start[0], x_min) and _same_z(end[0], x_min):
        return _range_within_bounds(start[1], end[1], y_min, y_max)
    if _same_z(start[0], x_max) and _same_z(end[0], x_max):
        return _range_within_bounds(start[1], end[1], y_min, y_max)
    if _same_z(start[1], y_min) and _same_z(end[1], y_min):
        return _range_within_bounds(start[0], end[0], x_min, x_max)
    if _same_z(start[1], y_max) and _same_z(end[1], y_max):
        return _range_within_bounds(start[0], end[0], x_min, x_max)
    return False


def _range_within_bounds(
    start: float,
    end: float,
    lower: float,
    upper: float,
) -> bool:
    return lower - _INSET_EPS_UM <= min(start, end) and max(start, end) <= (
        upper + _INSET_EPS_UM
    )


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

    simple_no_hole_refs = _simple_planar_inset_geometry_refs(
        gdstk,
        geometry_ref,
        breakpoints_um,
    )
    if simple_no_hole_refs:
        return simple_no_hole_refs

    simple_refs = _simple_hole_planar_inset_geometry_refs(
        gdstk,
        geometry_ref,
        breakpoints_um,
    )
    if simple_refs:
        return simple_refs

    refs: list[tuple[str, float | None, float | None, dict[str, Any]]] = []
    base_region = _gdstk_surface_region(geometry_ref)

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
                base_region=base_region,
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
            base_region=base_region,
            core_min_um=core_min_um,
        )
    )
    return tuple(refs)


def _simple_planar_inset_geometry_refs(
    gdstk: Any,
    geometry_ref: Mapping[str, Any],
    breakpoints_um: tuple[float, ...],
) -> tuple[tuple[str, float | None, float | None, dict[str, Any]], ...]:
    if geometry_ref.get("hole_loops"):
        return ()
    outer_loop = _clean_loop(geometry_ref["outer_loop"])
    refs: list[tuple[str, float | None, float | None, dict[str, Any]]] = []
    for band_min_um, band_max_um in zip(
        breakpoints_um,
        breakpoints_um[1:],
        strict=False,
    ):
        outer_min = _single_offset_loop(gdstk, outer_loop, -band_min_um)
        outer_max = _single_offset_loop(gdstk, outer_loop, -band_max_um)
        if outer_min is None or outer_max is None:
            return ()
        refs.append(
            (
                _inset_band_label(band_min_um, band_max_um),
                band_min_um,
                band_max_um,
                _geometry_ref_with_loops(geometry_ref, outer_min, (outer_max,)),
            )
        )
    core_min_um = breakpoints_um[-1]
    core_outer = _single_offset_loop(gdstk, outer_loop, -core_min_um)
    if core_outer is None:
        return ()
    refs.append(
        (
            _core_label(core_min_um),
            core_min_um,
            None,
            _geometry_ref_with_loops(geometry_ref, core_outer, ()),
        )
    )
    return tuple(refs)


def _simple_hole_planar_inset_geometry_refs(
    gdstk: Any,
    geometry_ref: Mapping[str, Any],
    breakpoints_um: tuple[float, ...],
) -> tuple[tuple[str, float | None, float | None, dict[str, Any]], ...]:
    hole_loops = tuple(_clean_loop(loop) for loop in geometry_ref.get("hole_loops", ()))
    if not hole_loops:
        return ()

    outer_loop = _clean_loop(geometry_ref["outer_loop"])
    refs: list[tuple[str, float | None, float | None, dict[str, Any]]] = []
    for band_min_um, band_max_um in zip(
        breakpoints_um,
        breakpoints_um[1:],
        strict=False,
    ):
        label = _inset_band_label(band_min_um, band_max_um)
        outer_min = _single_offset_loop(gdstk, outer_loop, -band_min_um)
        outer_max = _single_offset_loop(gdstk, outer_loop, -band_max_um)
        if outer_min is not None and outer_max is not None:
            refs.append(
                (
                    label,
                    band_min_um,
                    band_max_um,
                    _geometry_ref_with_loops(geometry_ref, outer_min, (outer_max,)),
                )
            )
        for hole_loop in hole_loops:
            hole_min = _single_offset_loop(gdstk, hole_loop, band_min_um)
            hole_max = _single_offset_loop(gdstk, hole_loop, band_max_um)
            if hole_min is None or hole_max is None:
                return ()
            refs.append(
                (
                    label,
                    band_min_um,
                    band_max_um,
                    _geometry_ref_with_loops(geometry_ref, hole_max, (hole_min,)),
                )
            )

    core_min_um = breakpoints_um[-1]
    core_outer = _single_offset_loop(gdstk, outer_loop, -core_min_um)
    core_holes = tuple(
        _single_offset_loop(gdstk, hole_loop, core_min_um)
        for hole_loop in hole_loops
    )
    if core_outer is None or any(loop is None for loop in core_holes):
        return ()
    refs.append(
        (
            _core_label(core_min_um),
            core_min_um,
            None,
            _geometry_ref_with_loops(
                geometry_ref,
                core_outer,
                tuple(loop for loop in core_holes if loop is not None),
            ),
        )
    )
    return tuple(refs)


def _single_offset_loop(
    gdstk: Any,
    loop: tuple[tuple[float, float], ...],
    offset_um: float,
) -> tuple[tuple[float, float], ...] | None:
    candidates = _offset_loop_candidates(gdstk, loop, offset_um)
    if len(candidates) != 1:
        return None
    return candidates[0]


def _planar_inset_band_refs(
    gdstk: Any,
    geometry_ref: Mapping[str, Any],
    *,
    base_region: tuple[Any, ...],
    band_min_um: float,
    band_max_um: float,
) -> tuple[dict[str, Any], ...]:
    inner_region = _offset_gdstk_region(gdstk, base_region, band_min_um)
    core_region = _offset_gdstk_region(gdstk, base_region, band_max_um)
    band_region = _boolean_gdstk_region(gdstk, inner_region, core_region, "not")
    return _geometry_refs_from_gdstk_region(geometry_ref, band_region)


def _planar_inset_core_refs(
    gdstk: Any,
    geometry_ref: Mapping[str, Any],
    *,
    base_region: tuple[Any, ...],
    core_min_um: float,
) -> tuple[dict[str, Any], ...]:
    core_region = _offset_gdstk_region(gdstk, base_region, core_min_um)
    return _geometry_refs_from_gdstk_region(geometry_ref, core_region)


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
    max_inset_um = limiting_um / 2.0
    valid_breakpoints = tuple(
        value
        for value in breakpoints_um
        if value == 0.0 or value < max_inset_um - _INSET_EPS_UM
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
        strip_ranges = (
            ("near", band_min_um, band_max_um),
            ("far", limiting_um - band_max_um, limiting_um - band_min_um),
        )
        for side, start_um, end_um in strip_ranges:
            quad_points = _sidewall_strip_points(
                points,
                width_um=width_um,
                height_um=height_um,
                axis=axis,
                start_um=start_um,
                end_um=end_um,
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
                        "sidewall_inset_rule": "short_axis_symmetric_strips",
                        "sidewall_inset_side": side,
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
        end_um=limiting_um - core_min_um,
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
                    "sidewall_inset_rule": "short_axis_symmetric_strips",
                    "sidewall_inset_side": "middle",
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


def _geometry_refs_from_gdstk_region(
    parent_geometry_ref: Mapping[str, Any],
    region: tuple[Any, ...],
) -> tuple[dict[str, Any], ...]:
    return tuple(
        _geometry_ref_from_gdstk_polygon(parent_geometry_ref, polygon)
        for polygon in _filter_gdstk_polygons(region)
    )


def _coordinate_2d_key(point: tuple[float, float]) -> tuple[float, float]:
    return tuple(round(float(value), 9) for value in point)


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
