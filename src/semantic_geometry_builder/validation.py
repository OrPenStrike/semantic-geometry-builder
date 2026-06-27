"""Fail-fast invariants for semantic geometry plans.

These are compiler checks, not backend repair passes. Pre-lowering validation
must catch duplicate points/curves/surfaces, T-junctions, missing interface
sources, bad volume closure, impossible surface use counts, and invalid tag
ledger mappings before Gmsh/OCC is asked to materialize anything.
"""

from __future__ import annotations

import json
from bisect import bisect_left, bisect_right
from collections import Counter
from collections.abc import Mapping
from typing import Any

from semantic_geometry_builder.models import (
    ROUTE_ALLOWED_REPRESENTATIONS,
    BackendEntityTagRecord,
    ConstructionBodyPlanRecord,
    CurvePlanRecord,
    CutHostOperationRecord,
    GeometryBuildInput,
    InterfacePlanRecord,
    MeshSizeHintRecord,
    PointPlanRecord,
    RouteLiteral,
    SemanticEntitySpec,
    SurfaceLoopRecord,
    SurfacePartitionRecord,
    SurfacePlanRecord,
    TagPlanRecord,
    VolumePlanRecord,
)

_INTERFACE_KINDS = {"MM", "SS", "AA", "MS", "MA", "SA"}
# gdstk booleans can leave tiny numerical slivers at curved mask boundaries;
# topology-significant overlaps are orders of magnitude larger than this.
_BOOLEAN_AREA_EPS_UM2 = 1e-8


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
    covered_surface_ids = {
        *surface_ids,
        *(
            surface.parent_surface_id
            for surface in surfaces
            if surface.parent_surface_id is not None
        ),
    }
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
            if surface_id not in covered_surface_ids
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

    missing_operation_surfaces = sorted(operation_surface_ids - covered_surface_ids)
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
    """Fail fast when backend-live volumes have no planned boundaries."""
    del route
    missing = sorted(
        volume.volume_id
        for volume in volumes
        if not volume.construction_only and not volume.surface_refs
    )
    if missing:
        raise ValueError(
            "Backend-live volumes require planned boundary surfaces: "
            f"{missing!r}"
        )


def validate_curve_plan_coverage(
    *,
    points: tuple[PointPlanRecord, ...],
    curves: tuple[CurvePlanRecord, ...],
    surface_loops: tuple[SurfaceLoopRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Validate canonical curve/loop references when the plan provides them."""
    errors: list[str] = []
    if any(not surface.construction_only for surface in surfaces) and (
        not points or not curves or not surface_loops
    ):
        errors.append(
            "backend-live surfaces require canonical points, curves, and loops"
        )

    point_ids: set[str] = set()
    point_signatures: dict[tuple[float, float, float], str] = {}
    points_by_id: dict[str, PointPlanRecord] = {}
    for point in points:
        if point.point_id in point_ids:
            errors.append(f"duplicate point_id: {point.point_id}")
        point_ids.add(point.point_id)
        points_by_id[point.point_id] = point
        signature = _coordinate_signature(point.coordinate)
        existing = point_signatures.get(signature)
        if existing is not None:
            errors.append(
                f"duplicate point coordinate: {existing} and {point.point_id}"
            )
        point_signatures[signature] = point.point_id

    curve_ids: set[str] = set()
    curve_signatures: dict[tuple[str, str], str] = {}
    for curve in curves:
        if curve.curve_id in curve_ids:
            errors.append(f"duplicate curve_id: {curve.curve_id}")
        curve_ids.add(curve.curve_id)
        if curve.start_point_id not in point_ids:
            errors.append(
                f"{curve.curve_id} references unknown point "
                f"{curve.start_point_id}"
            )
        if curve.end_point_id not in point_ids:
            errors.append(
                f"{curve.curve_id} references unknown point {curve.end_point_id}"
            )
        if curve.start_point_id == curve.end_point_id:
            errors.append(f"{curve.curve_id} has identical start/end points")
            continue
        signature = tuple(sorted((curve.start_point_id, curve.end_point_id)))
        existing = curve_signatures.get(signature)
        if existing is not None:
            errors.append(
                f"duplicate curve geometry: {existing} and {curve.curve_id}"
            )
        curve_signatures[signature] = curve.curve_id

    point_axis_index = _point_axis_index(points_by_id)
    point_line_index = _axis_aligned_point_index(points_by_id)
    for curve in curves:
        start = points_by_id.get(curve.start_point_id)
        end = points_by_id.get(curve.end_point_id)
        if start is None or end is None:
            continue
        interior_points = [
            point_id
            for point_id in _segment_candidate_point_ids(
                start.coordinate,
                end.coordinate,
                point_axis_index,
                point_line_index,
            )
            if point_id not in {curve.start_point_id, curve.end_point_id}
            and _point_on_segment_interior(
                points_by_id[point_id].coordinate,
                start.coordinate,
                end.coordinate,
            )
        ]
        if interior_points:
            errors.append(
                f"{curve.curve_id} contains planned points in its interior: "
                f"{interior_points!r}"
            )

    loop_ids = {loop.loop_id for loop in surface_loops}
    curves_by_id = {curve.curve_id: curve for curve in curves}
    for loop in surface_loops:
        if len(loop.curve_refs) < 3:
            errors.append(f"{loop.loop_id} requires at least three curves")
            continue
        directed_edges: list[tuple[str, str]] = []
        for curve_ref in loop.curve_refs:
            curve = curves_by_id.get(curve_ref.curve_id)
            if curve is None:
                errors.append(
                    f"{loop.loop_id} references unknown curve {curve_ref.curve_id}"
                )
                continue
            if curve_ref.orientation == 1:
                directed_edges.append((curve.start_point_id, curve.end_point_id))
            else:
                directed_edges.append((curve.end_point_id, curve.start_point_id))
        for index, (_, end_point_id) in enumerate(directed_edges):
            next_start_id, _ = directed_edges[(index + 1) % len(directed_edges)]
            if end_point_id != next_start_id:
                errors.append(f"{loop.loop_id} curve refs do not form a closed loop")
                break

    for surface in surfaces:
        if surface.construction_only:
            continue
        if surface.outer_loop_ref is None:
            errors.append(f"{surface.surface_id} lacks outer_loop_ref")
        refs = (
            *((surface.outer_loop_ref,) if surface.outer_loop_ref is not None else ()),
            *surface.hole_loop_refs,
        )
        for loop_ref in refs:
            if loop_ref not in loop_ids:
                errors.append(
                    f"{surface.surface_id} references unknown loop {loop_ref}"
                )

    if errors:
        raise ValueError("Invalid curve plan coverage: " + "; ".join(errors))


def validate_interface_surface_source_of_truth(
    *,
    interfaces: tuple[InterfacePlanRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Require every solver-relevant interface surface to come from InterfacePlan."""
    interface_ids = {interface.interface_id for interface in interfaces}
    errors: list[str] = []
    for surface in surfaces:
        if surface.construction_only:
            continue
        interface_kinds = _surface_interface_kinds(surface)
        if len(interface_kinds) > 2:
            errors.append(
                f"{surface.surface_id} has too many interface kinds "
                f"{interface_kinds!r}"
            )
        if interface_kinds and surface.interface_id is None:
            errors.append(
                f"{surface.surface_id} has interface kinds "
                f"{interface_kinds!r} without InterfacePlan source"
            )
        if (
            surface.interface_id is not None
            and surface.interface_id not in interface_ids
        ):
            errors.append(
                f"{surface.surface_id} references unknown interface "
                f"{surface.interface_id}"
            )
    if errors:
        raise ValueError("Invalid interface source of truth: " + "; ".join(errors))


def validate_surface_deduplication(
    *,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Reject two backend-live surfaces with the same canonical signature."""
    signatures: dict[str, str] = {}
    errors: list[str] = []
    for surface in surfaces:
        if surface.construction_only:
            continue
        signature = _surface_signature(surface)
        existing = signatures.get(signature)
        if existing is not None and existing != surface.surface_id:
            errors.append(
                f"duplicate live surface geometry: {existing} and "
                f"{surface.surface_id}"
            )
        signatures[signature] = surface.surface_id
    if errors:
        raise ValueError("Invalid surface deduplication: " + "; ".join(errors))


def validate_no_surface_overlap(
    *,
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Reject horizontal same-plane partial overlap for all solver-live surfaces.

    This guard is intentionally not allowed to skip high-count conductors by
    semantic owner. Physical groups may collapse many instances into one solver
    label, but topology validation must still see every live surface. General
    sidewall/arbitrary-plane overlap validation is a separate contract and must
    fail fast before those inset cases are advertised as mesh-safe.
    """
    horizontal_surfaces = tuple(
        (surface, z_um, region, _region_bounds(region))
        for surface in surfaces
        if not surface.construction_only
        for z_um, region in (_horizontal_surface_region(surface),)
        if region
    )
    surfaces_by_z = {}
    for surface, z_um, region, bounds in horizontal_surfaces:
        surfaces_by_z.setdefault(round(z_um, 9), []).append((surface, region, bounds))

    errors: list[str] = []
    for z_um, z_surfaces in surfaces_by_z.items():
        ordered = sorted(z_surfaces, key=lambda item: item[2][0])
        for index, (left, left_region, left_bounds) in enumerate(ordered):
            for right, right_region, right_bounds in ordered[index + 1 :]:
                if right_bounds[0] > left_bounds[2] + 1e-9:
                    break
                if not _bounds_overlap(left_bounds, right_bounds):
                    continue
                if not _regions_overlap(left_region, right_region):
                    continue
                errors.append(
                    f"{left.surface_id} overlaps {right.surface_id} on z={z_um}"
                )
                break
            if errors:
                break
        if errors:
            break
    if errors:
        raise ValueError("Invalid surface overlap: " + "; ".join(errors))


def validate_inset_mesh_contract(
    *,
    surface_partitions: tuple[SurfacePartitionRecord, ...],
    mesh_size_hints: tuple[MeshSizeHintRecord, ...],
) -> None:
    """Require every finite inset band to export a downstream mesh-size hint.

    The planner may represent horizontal bands with planar loops and sidewall
    bands with explicit quad strips. Both are live topology; validation only
    cares that finite bands carry mesh-size hints fine enough to resolve the
    requested inset width.
    """
    hints_by_partition = {
        hint.source_partition_id: hint
        for hint in mesh_size_hints
        if hint.source_partition_id is not None
    }
    errors: list[str] = []
    for hint in mesh_size_hints:
        if hint.max_size_um <= 0:
            errors.append(f"{hint.target_id} mesh hint max_size_um must be positive")

    for partition in surface_partitions:
        if partition.band_min_um is None or partition.band_max_um is None:
            continue
        width_um = partition.band_max_um - partition.band_min_um
        if width_um <= 0:
            continue
        hint = hints_by_partition.get(partition.partition_id)
        if hint is None:
            errors.append(f"{partition.partition_id} finite inset band lacks mesh hint")
            continue
        if hint.target_id != partition.child_surface_id:
            errors.append(
                f"{partition.partition_id} mesh hint targets {hint.target_id}, "
                f"expected {partition.child_surface_id}"
            )
        if hint.max_size_um > (width_um / 2.0) + 1e-12:
            errors.append(
                f"{partition.partition_id} mesh hint {hint.max_size_um} um is "
                f"too coarse for {width_um} um inset band"
            )
    if errors:
        raise ValueError("Invalid inset mesh contract: " + "; ".join(errors))


def validate_volume_surface_closure(
    *,
    volumes: tuple[VolumePlanRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
    surface_loops: tuple[SurfaceLoopRecord, ...],
) -> None:
    """Validate volume surface references before backend lowering.

    Full closed-shell validation belongs to the canonical curve/loop plan. This
    guard rejects missing references and then checks curve incidence from the
    planned loops, before OCC is allowed to construct a volume.
    """
    surfaces_by_id = {surface.surface_id: surface for surface in surfaces}
    loops_by_id = {loop.loop_id: loop for loop in surface_loops}
    errors: list[str] = []
    for volume in volumes:
        if volume.construction_only:
            continue
        curve_use_count: Counter[str] = Counter()
        for surface_ref in volume.surface_refs:
            surface = surfaces_by_id.get(surface_ref.surface_id)
            if surface is None:
                errors.append(
                    f"{volume.volume_id} references unknown surface "
                    f"{surface_ref.surface_id}"
                )
                continue
            loop_ids = (
                *((surface.outer_loop_ref,) if surface.outer_loop_ref else ()),
                *surface.hole_loop_refs,
            )
            if not loop_ids:
                errors.append(
                    f"{volume.volume_id} uses non-canonical surface "
                    f"{surface.surface_id}"
                )
            for loop_id in loop_ids:
                loop = loops_by_id.get(loop_id)
                if loop is None:
                    errors.append(
                        f"{volume.volume_id} references unknown loop {loop_id}"
                    )
                    continue
                curve_use_count.update(
                    curve_ref.curve_id for curve_ref in loop.curve_refs
                )
        bad_counts = {
            curve_id: count
            for curve_id, count in curve_use_count.items()
            if count != 2
        }
        if bad_counts:
            errors.append(
                f"{volume.volume_id} shell is not closed; curve incidence "
                f"{bad_counts!r}"
            )
    if errors:
        raise ValueError("Invalid volume surface closure: " + "; ".join(errors))


def validate_surface_use_counts(
    *,
    volumes: tuple[VolumePlanRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Validate internal/exterior surface incidence across live volumes."""
    live_surface_ids = {
        surface.surface_id
        for surface in surfaces
        if not surface.construction_only
    }
    use_counts: Counter[str] = Counter(
        surface_ref.surface_id
        for volume in volumes
        if not volume.construction_only
        for surface_ref in volume.surface_refs
    )
    errors: list[str] = []
    for surface in surfaces:
        if surface.construction_only:
            continue
        count = use_counts[surface.surface_id]
        if surface.surface_id not in live_surface_ids:
            errors.append(f"{surface.surface_id} is not live")
        if surface.interface_id is not None and count not in {1, 2}:
            errors.append(
                f"{surface.surface_id} interface use-count must be 1 or 2, got "
                f"{count}"
            )
        elif surface.interface_id is None and count != 1:
            errors.append(
                f"{surface.surface_id} exterior use-count must be 1, got {count}"
            )
    if errors:
        raise ValueError("Invalid surface use counts: " + "; ".join(errors))


def validate_surface_partition_coverage(
    *,
    interfaces: tuple[InterfacePlanRecord, ...],
    surface_partitions: tuple[SurfacePartitionRecord, ...],
    surfaces: tuple[SurfacePlanRecord, ...],
) -> None:
    """Validate that partition records point to real parent/child plan ids."""
    surfaces_by_id = {surface.surface_id: surface for surface in surfaces}
    interface_ids = {interface.interface_id for interface in interfaces}
    interface_ids.update(
        surface.interface_id
        for surface in surfaces
        if surface.interface_id is not None
    )
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
        if (
            "outer_loop" not in geometry_ref
            and "loop_geometry_ref" not in geometry_ref
            and "quad_points" not in geometry_ref
        ):
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
        core_min_values = {partition.band_min_um for partition in core_partitions}
        if not core_partitions or len(core_min_values) != 1:
            errors.append(
                f"{parent_interface_id} inset children require one core band"
            )
            continue
        expected_min = 0.0
        band_ranges = sorted(
            {
                (partition.band_min_um, partition.band_max_um)
                for partition in band_partitions
            },
            key=lambda item: item[0] if item[0] is not None else -1.0,
        )
        for band_min_um, band_max_um in band_ranges:
            if band_min_um is None:
                errors.append(f"{parent_interface_id} band_min_um is required")
                continue
            if band_min_um != expected_min:
                errors.append(
                    f"{parent_interface_id} band starts at {band_min_um}, "
                    f"expected {expected_min}"
                )
            if (
                band_max_um is None
                or band_max_um <= band_min_um
            ):
                errors.append(f"{parent_interface_id} has invalid band range")
                continue
            expected_min = band_max_um
        core_min = next(iter(core_min_values))
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
    """Validate that every exported geometry record has a tag plan.

    Boundary surfaces can be backend-live topology used to close volumes without
    being exported as standalone surface physical groups. Interface surfaces and
    non-construction volumes must have `TagPlanRecord`s before OCC assigns
    dim-tags.
    """
    live_sources = {
        ("surface", surface.surface_id)
        for surface in surfaces
        if not surface.construction_only and surface.interface_id is not None
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


def validate_backend_tag_ledger(
    *,
    backend_tags: tuple[BackendEntityTagRecord, ...],
) -> None:
    """Validate plan-id to backend-tag mapping after OCC lowering."""
    source_to_dimtags: dict[tuple[str, str], set[tuple[int, int]]] = {}
    dimtag_to_sources: dict[tuple[int, int], set[tuple[str, str]]] = {}
    for tag in backend_tags:
        source = (tag.source_record_kind, tag.source_record_id)
        source_to_dimtags.setdefault(source, set()).add(tag.dim_tag)
        dimtag_to_sources.setdefault(tag.dim_tag, set()).add(source)

    errors: list[str] = []
    for source, dimtags in source_to_dimtags.items():
        if len(dimtags) > 1:
            errors.append(f"{source!r} maps to multiple backend tags {dimtags!r}")
    for dimtag, sources in dimtag_to_sources.items():
        if len(sources) > 1:
            errors.append(f"backend tag {dimtag!r} is shared by sources {sources!r}")
    if errors:
        raise ValueError("Invalid backend tag ledger: " + "; ".join(errors))


def _horizontal_surface_region(
    surface: SurfacePlanRecord,
) -> tuple[float, tuple[Any, ...]]:
    geometry_ref = surface.geometry_ref
    if "outer_loop" not in geometry_ref or "quad_points" in geometry_ref:
        return 0.0, ()
    import gdstk

    outer = gdstk.Polygon(_clean_loop2d(geometry_ref["outer_loop"]))
    holes = tuple(
        gdstk.Polygon(_clean_loop2d(hole_loop))
        for hole_loop in geometry_ref.get("hole_loops", ())
    )
    if holes:
        region = tuple(gdstk.boolean((outer,), holes, "not", precision=1e-9) or ())
    else:
        region = (outer,)
    return _geometry_ref_z_um(geometry_ref), tuple(
        polygon
        for polygon in region
        if abs(float(polygon.area())) > _BOOLEAN_AREA_EPS_UM2
    )


def _regions_overlap(left: tuple[Any, ...], right: tuple[Any, ...]) -> bool:
    import gdstk

    overlap = gdstk.boolean(left, right, "and", precision=1e-9) or ()
    return any(abs(polygon.area()) > _BOOLEAN_AREA_EPS_UM2 for polygon in overlap)


def _coordinate_2d_key(point: tuple[float, float]) -> tuple[float, float]:
    return (round(float(point[0]), 9), round(float(point[1]), 9))


def _region_bounds(region: tuple[Any, ...]) -> tuple[float, float, float, float]:
    boxes = [polygon.bounding_box() for polygon in region]
    return (
        min(float(box[0][0]) for box in boxes),
        min(float(box[0][1]) for box in boxes),
        max(float(box[1][0]) for box in boxes),
        max(float(box[1][1]) for box in boxes),
    )


def _bounds_overlap(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> bool:
    return not (
        left[2] <= right[0] + 1e-9
        or right[2] <= left[0] + 1e-9
        or left[3] <= right[1] + 1e-9
        or right[3] <= left[1] + 1e-9
    )


def _clean_loop2d(loop: Any) -> tuple[tuple[float, float], ...]:
    points = tuple((float(point[0]), float(point[1])) for point in loop)
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("surface loop requires at least 3 points")
    return points


def _geometry_ref_z_um(geometry_ref: Mapping[str, Any]) -> float:
    z_min_um = float(geometry_ref.get("z_min_um", geometry_ref.get("z_um", 0.0)))
    if geometry_ref.get("shell_part") == "top":
        return z_min_um + float(geometry_ref.get("thickness_um", 0.0))
    if geometry_ref.get("shell_part") == "bottom":
        return z_min_um
    plane = geometry_ref.get("plane") or geometry_ref.get("contact_plane")
    if isinstance(plane, Mapping) and plane.get("axis") == "z":
        return float(plane["value_um"])
    return z_min_um


def _surface_signature(surface: SurfacePlanRecord) -> str:
    if surface.outer_loop_ref is not None:
        return json.dumps(
            {
                "outer": surface.outer_loop_ref,
                "holes": surface.hole_loop_refs,
            },
            sort_keys=True,
        )
    return json.dumps(surface.geometry_ref, sort_keys=True, default=str)


def _surface_interface_kinds(surface: SurfacePlanRecord) -> tuple[str, ...]:
    kinds = surface.metadata.get("interface_kinds", ())
    if isinstance(kinds, str):
        kinds = (kinds,)
    return tuple(sorted({str(kind) for kind in kinds if str(kind) in _INTERFACE_KINDS}))


def _coordinate_signature(
    coordinate: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(round(float(value), 9) for value in coordinate)


def _point_axis_index(
    points_by_id: Mapping[str, PointPlanRecord],
) -> tuple[tuple[tuple[float, ...], tuple[str, ...]], ...]:
    indexes: list[tuple[tuple[float, ...], tuple[str, ...]]] = []
    for axis in range(3):
        items = sorted(
            (
                _coordinate_signature(point.coordinate)[axis],
                point_id,
            )
            for point_id, point in points_by_id.items()
        )
        indexes.append(
            (
                tuple(value for value, _ in items),
                tuple(point_id for _, point_id in items),
            )
        )
    return tuple(indexes)


def _axis_aligned_point_index(
    points_by_id: Mapping[str, PointPlanRecord],
) -> dict[tuple[int, tuple[float, float]], tuple[tuple[float, ...], tuple[str, ...]]]:
    records: dict[
        tuple[int, tuple[float, float]],
        list[tuple[float, str]],
    ] = {}
    for point_id, point in points_by_id.items():
        key = _coordinate_signature(point.coordinate)
        for varying_axis in range(3):
            fixed_key = tuple(
                key[axis]
                for axis in range(3)
                if axis != varying_axis
            )
            records.setdefault((varying_axis, fixed_key), []).append(
                (key[varying_axis], point_id)
            )
    return {
        line_key: (
            tuple(value for value, _ in sorted_items),
            tuple(point_id for _, point_id in sorted_items),
        )
        for line_key, items in records.items()
        for sorted_items in (tuple(sorted(items)),)
    }


def _segment_candidate_point_ids(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    point_axis_index: tuple[tuple[tuple[float, ...], tuple[str, ...]], ...],
    point_line_index: Mapping[
        tuple[int, tuple[float, float]],
        tuple[tuple[float, ...], tuple[str, ...]],
    ],
) -> tuple[str, ...]:
    start_key = _coordinate_signature(start)
    end_key = _coordinate_signature(end)
    varying_axes = tuple(
        axis
        for axis in range(3)
        if abs(start_key[axis] - end_key[axis]) > 1e-9
    )
    if len(varying_axes) == 1:
        varying_axis = varying_axes[0]
        fixed_key = tuple(
            start_key[axis]
            for axis in range(3)
            if axis != varying_axis
        )
        values, point_ids = point_line_index.get(
            (varying_axis, fixed_key),
            ((), ()),
        )
        lower = min(start_key[varying_axis], end_key[varying_axis]) - 1e-9
        upper = max(start_key[varying_axis], end_key[varying_axis]) + 1e-9
        left = bisect_left(values, lower)
        right = bisect_right(values, upper)
        return point_ids[left:right]
    bounds = tuple(
        (
            min(start_key[axis], end_key[axis]) - 1e-9,
            max(start_key[axis], end_key[axis]) + 1e-9,
        )
        for axis in range(3)
    )
    ranges: list[tuple[int, int, int]] = []
    for axis, (values, _) in enumerate(point_axis_index):
        lower, upper = bounds[axis]
        left = bisect_left(values, lower)
        right = bisect_right(values, upper)
        ranges.append((right - left, left, axis))
    count, left, axis = min(ranges)
    return point_axis_index[axis][1][left : left + count]


def _point_on_segment_interior(
    point: tuple[float, float, float],
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> bool:
    vector = tuple(float(end[index]) - float(start[index]) for index in range(3))
    offset = tuple(float(point[index]) - float(start[index]) for index in range(3))
    length_sq = sum(value * value for value in vector)
    if length_sq <= 1e-18:
        return False
    parameter = sum(offset[index] * vector[index] for index in range(3)) / length_sq
    if parameter <= 1e-9 or parameter >= 1.0 - 1e-9:
        return False
    closest = tuple(
        float(start[index]) + parameter * vector[index]
        for index in range(3)
    )
    distance_sq = sum((float(point[index]) - closest[index]) ** 2 for index in range(3))
    return distance_sq <= 1e-18


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
