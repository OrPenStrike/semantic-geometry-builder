"""Machine-checkable Engine Gates for conformal geometry claims.

An Engine Gate is the evidence boundary between "the compiler produced a
reviewable plan" and "SGB can claim this route output is conformal geometry."
These reports are stricter than ordinary validation because they name the
specific engine-level claim being made, list the records that were checked, and
fail loudly when the claim is not machine-checkable.

The three SGB-owned gates are intentionally small:

- `2d_inset_coverage` proves inset child surfaces exactly replace their parent
  surface in 2D footprint space.
- `volume_adjacency_conformality` proves planned live surfaces are referenced by
  the expected one or two volumes.
- `gmsh_brep_conformality` proves the lowered Gmsh BRep boundaries still match
  planned curves and surfaces after `occ.synchronize()`.

Downstream tetra mesh topology remains a fourth gate owned by the meshing
consumer. Passing these reports means SGB has produced conformal CAD topology;
it does not mean a particular mesh-size strategy is solver-ready.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from math import sqrt
from typing import Any

from semantic_geometry_builder.models import ConstructionPlanRecord

_COORD_TOL_UM = 1e-9
_AREA_TOL_UM2 = 1e-8


def assert_engine_gate_pass(report: Mapping[str, Any]) -> None:
    """Fail the build when an Engine Gate report does not pass.

    Engine Gate failures are not warnings. If a report is missing or has
    `status != "pass"`, the route output may still be useful for debugging, but
    it must not be treated as proven conformal geometry.
    """
    if not isinstance(report, Mapping):
        raise ValueError("Engine Gate report is missing")
    if report.get("status") != "pass":
        failures = report.get("failures", ())
        raise ValueError(
            f"Engine Gate {report.get('engine_gate')} failed: {failures!r}"
        )


def engine_gate_2d_inset_coverage(plan: ConstructionPlanRecord) -> dict[str, Any]:
    """Prove inset child surfaces exactly replace each logical parent surface.

    This gate works before OCC lowering. It compares the parent footprint saved
    before `apply_inset_surface_partitions()` against the union of generated
    child ring/core footprints. A passing report means no live parent surface
    remains, child surfaces have nonzero area, child union covers the parent,
    and children do not overlap or leak outside the parent footprint.
    """
    surfaces_by_id = {surface.surface_id: surface for surface in plan.surfaces}
    parent_geometry_refs = plan.metadata.get("inset_parent_geometry_refs", {})
    failures: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []

    partitions_by_parent: dict[str, list[Any]] = {}
    for partition in plan.surface_partitions:
        parent_id = str(partition.metadata.get("parent_surface_id", ""))
        if parent_id:
            partitions_by_parent.setdefault(parent_id, []).append(partition)

    for parent_surface_id, partitions in sorted(partitions_by_parent.items()):
        live_parent = surfaces_by_id.get(parent_surface_id)
        if live_parent is not None and not live_parent.construction_only:
            failures.append(
                _failure(
                    "parent_surface_still_live",
                    [parent_surface_id],
                    "Inset children must replace the parent as live geometry.",
                )
            )
        parent_geometry_ref = parent_geometry_refs.get(parent_surface_id)
        if parent_geometry_ref is None:
            failures.append(
                _failure(
                    "missing_parent_footprint",
                    [parent_surface_id],
                    "Inset coverage cannot be checked without parent geometry.",
                )
            )
            continue
        parent_region = _surface_region(parent_geometry_ref)
        if not parent_region:
            failures.append(
                _failure(
                    "unsupported_parent_geometry",
                    [parent_surface_id],
                    "Parent inset footprint is not representable as a 2D region.",
                )
            )
            continue

        child_regions = []
        child_ids = []
        min_child_area_um2: float | None = None
        for partition in partitions:
            child = surfaces_by_id.get(partition.child_surface_id)
            if child is None:
                failures.append(
                    _failure(
                        "missing_child_surface",
                        [partition.partition_id, partition.child_surface_id],
                        "SurfacePartitionRecord references a missing child surface.",
                    )
                )
                continue
            child_region = _surface_region(child.geometry_ref, parent_geometry_ref)
            child_area = _region_area(child_region)
            min_child_area_um2 = (
                child_area
                if min_child_area_um2 is None
                else min(min_child_area_um2, child_area)
            )
            if child_area <= _AREA_TOL_UM2:
                failures.append(
                    _failure(
                        "child_sliver",
                        [partition.partition_id, child.surface_id],
                        f"Child inset surface area {child_area} is below tolerance.",
                    )
                )
            child_regions.extend(child_region)
            child_ids.append(child.surface_id)

        if not child_regions:
            failures.append(
                _failure(
                    "missing_child_regions",
                    [parent_surface_id],
                    "Inset parent has no checkable child regions.",
                )
            )
            continue

        union_region = _boolean_region(child_regions, (), "or")
        parent_area = _region_area(parent_region)
        child_area_sum = sum(_region_area((region,)) for region in child_regions)
        union_area = _region_area(union_region)
        gap_area = _region_area(_boolean_region(parent_region, union_region, "not"))
        outside_area = _region_area(_boolean_region(union_region, parent_region, "not"))
        overlap_area = max(0.0, child_area_sum - union_area)
        relative_area_error = (
            abs(parent_area - union_area) / parent_area
            if parent_area > _AREA_TOL_UM2
            else 0.0
        )
        status = (
            "pass"
            if (
                gap_area <= _AREA_TOL_UM2
                and outside_area <= _AREA_TOL_UM2
                and overlap_area <= _AREA_TOL_UM2
                and relative_area_error <= 1e-9
            )
            else "fail"
        )
        if status == "fail":
            failures.append(
                _failure(
                    "inset_coverage_mismatch",
                    [parent_surface_id, *child_ids],
                    "Inset children do not exactly cover the parent footprint.",
                )
            )
        records.append(
            {
                "parent_surface_id": parent_surface_id,
                "child_surface_ids": child_ids,
                "parent_area_um2": parent_area,
                "child_area_sum_um2": child_area_sum,
                "union_area_um2": union_area,
                "gap_area_um2": gap_area,
                "outside_area_um2": outside_area,
                "overlap_area_um2": overlap_area,
                "relative_area_error": relative_area_error,
                "min_child_area_um2": min_child_area_um2,
                "status": status,
            }
        )

    return _report(
        plan,
        "2d_inset_coverage",
        "after_build_route_construction_plan",
        failures,
        records,
        counts={"parents": len(partitions_by_parent)},
    )


def engine_gate_volume_adjacency_conformality(
    plan: ConstructionPlanRecord,
) -> dict[str, Any]:
    """Prove every live planned surface has the expected volume incidence.

    This is the plan-level shared-face contract. Each live surface must declare
    `metadata["boundary_volume_ids"]`, and the volumes that actually reference
    that surface must match those ids exactly. Exterior faces have one adjacent
    volume; retained internal interfaces have two. More than two adjacent
    volumes is a non-manifold topology failure.
    """
    volume_refs_by_surface: dict[str, list[str]] = {}
    failures: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []

    for volume in plan.volumes:
        seen: set[str] = set()
        for surface_ref in volume.surface_refs:
            if surface_ref.surface_id in seen:
                failures.append(
                    _failure(
                        "duplicate_surface_ref_in_volume",
                        [volume.volume_id, surface_ref.surface_id],
                        "A volume references the same surface more than once.",
                    )
                )
            seen.add(surface_ref.surface_id)
            volume_refs_by_surface.setdefault(surface_ref.surface_id, []).append(
                volume.owner_semantic_id
            )

    for surface in plan.surfaces:
        if surface.construction_only:
            continue
        expected = tuple(
            str(value)
            for value in surface.metadata.get("boundary_volume_ids", ())
        )
        actual = tuple(sorted(volume_refs_by_surface.get(surface.surface_id, ())))
        expected_sorted = tuple(sorted(expected))
        expected_count = len(expected_sorted)
        actual_count = len(actual)
        status = "pass"
        if not expected_sorted:
            status = "fail"
            failures.append(
                _failure(
                    "missing_boundary_volume_ids",
                    [surface.surface_id],
                    "Live surface lacks explicit boundary_volume_ids metadata.",
                )
            )
        elif expected_count not in {1, 2}:
            status = "fail"
            failures.append(
                _failure(
                    "invalid_expected_volume_count",
                    [surface.surface_id],
                    "Expected surface incidence must be one or two volumes.",
                )
            )
        elif actual_count != expected_count or actual != expected_sorted:
            status = "fail"
            failures.append(
                _failure(
                    "volume_adjacency_mismatch",
                    [surface.surface_id],
                    "Actual volume owners do not match boundary_volume_ids.",
                )
            )
        if actual_count > 2:
            status = "fail"
            failures.append(
                _failure(
                    "surface_used_by_more_than_two_volumes",
                    [surface.surface_id],
                    "A live surface is referenced by more than two volumes.",
                )
            )
        records.append(
            {
                "surface_id": surface.surface_id,
                "physical_name": surface.metadata.get("physical_name"),
                "interface_type": "_".join(surface.metadata.get("interface_kinds", ())),
                "expected_adjacent_volume_ids": expected_sorted,
                "actual_adjacent_volume_ids": actual,
                "volume_use_count": actual_count,
                "expected_use_count": expected_count,
                "status": status,
            }
        )

    return _report(
        plan,
        "volume_adjacency_conformality",
        "after_build_route_construction_plan",
        failures,
        records,
        counts={"surfaces": len(records)},
    )


def engine_gate_gmsh_brep_conformality(
    plan: ConstructionPlanRecord,
    *,
    gmsh: Any,
    source_tags: Mapping[tuple[str, str], Sequence[tuple[int, int]]],
    curve_tags: Mapping[str, int],
) -> dict[str, Any]:
    """Prove lowered Gmsh BRep topology still matches the compiler plan.

    This gate runs after `gmsh.model.occ.synchronize()`, when live backend tags
    exist. It checks that each planned surface maps to exactly one Gmsh surface,
    each planned volume maps to exactly one Gmsh volume, surface boundaries use
    the planned curve tags, and volume boundaries use the planned surface tags.
    Coordinate coincidence is not enough: this gate is about live BRep tags.
    """
    failures: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []
    loops_by_id = {loop.loop_id: loop for loop in plan.surface_loops}
    surface_tag_by_id = _single_tag_map(source_tags, "surface", failures)
    volume_tag_by_id = _single_tag_map(source_tags, "volume", failures)

    for surface in plan.surfaces:
        if surface.construction_only:
            continue
        surface_tag = surface_tag_by_id.get(surface.surface_id)
        if surface_tag is None:
            failures.append(
                _failure(
                    "missing_backend_surface_tag",
                    [surface.surface_id],
                    "Live surface has no lowered Gmsh surface tag.",
                )
            )
            continue
        expected_curves = _planned_surface_curve_tags(
            surface,
            loops_by_id,
            curve_tags,
        )
        actual_curves = {
            tag
            for dim, tag in gmsh.model.getBoundary(
                [(2, surface_tag)],
                oriented=False,
                recursive=False,
            )
            if dim == 1
        }
        status = "pass" if actual_curves == expected_curves else "fail"
        if status == "fail":
            failures.append(
                _failure(
                    "surface_boundary_curve_mismatch",
                    [surface.surface_id],
                    "Gmsh surface boundary curves do not match planned curves.",
                )
            )
        records.append(
            {
                "source_record_kind": "surface",
                "source_record_id": surface.surface_id,
                "backend_tag": surface_tag,
                "expected_curve_tags": sorted(expected_curves),
                "actual_curve_tags": sorted(actual_curves),
                "status": status,
            }
        )

    for volume in plan.volumes:
        if volume.construction_only:
            continue
        volume_tag = volume_tag_by_id.get(volume.volume_id)
        if volume_tag is None:
            failures.append(
                _failure(
                    "missing_backend_volume_tag",
                    [volume.volume_id],
                    "Live volume has no lowered Gmsh volume tag.",
                )
            )
            continue
        expected_surfaces = {
            surface_tag_by_id[surface_ref.surface_id]
            for surface_ref in volume.surface_refs
            if surface_ref.surface_id in surface_tag_by_id
        }
        actual_surfaces = {
            tag
            for dim, tag in gmsh.model.getBoundary(
                [(3, volume_tag)],
                oriented=False,
                recursive=False,
            )
            if dim == 2
        }
        status = "pass" if actual_surfaces == expected_surfaces else "fail"
        if status == "fail":
            failures.append(
                _failure(
                    "volume_boundary_surface_mismatch",
                    [volume.volume_id],
                    "Gmsh volume boundary surfaces do not match planned surfaces.",
                )
            )
        records.append(
            {
                "source_record_kind": "volume",
                "source_record_id": volume.volume_id,
                "backend_tag": volume_tag,
                "expected_surface_tags": sorted(expected_surfaces),
                "actual_surface_tags": sorted(actual_surfaces),
                "status": status,
            }
        )

    return _report(
        plan,
        "gmsh_brep_conformality",
        "after_occ_synchronize",
        failures,
        records,
        counts={
            "surfaces": len(surface_tag_by_id),
            "volumes": len(volume_tag_by_id),
        },
    )


def _single_tag_map(
    source_tags: Mapping[tuple[str, str], Sequence[tuple[int, int]]],
    source_kind: str,
    failures: list[dict[str, Any]],
) -> dict[str, int]:
    records: dict[str, int] = {}
    dimtags_seen: dict[tuple[int, int], str] = {}
    for (kind, source_id), dimtags in source_tags.items():
        if kind != source_kind:
            continue
        if len(dimtags) != 1:
            failures.append(
                _failure(
                    "source_maps_to_multiple_backend_tags",
                    [source_id],
                    "A live source id must map to exactly one backend dim-tag.",
                )
            )
            continue
        dimtag = tuple(dimtags[0])
        existing = dimtags_seen.get(dimtag)
        if existing is not None:
            failures.append(
                _failure(
                    "backend_tag_shared_by_sources",
                    [existing, source_id],
                    "A backend dim-tag is shared by multiple source ids.",
                )
            )
        dimtags_seen[dimtag] = source_id
        records[source_id] = int(dimtag[1])
    return records


def _planned_surface_curve_tags(
    surface: Any,
    loops_by_id: Mapping[str, Any],
    curve_tags: Mapping[str, int],
) -> set[int]:
    curve_ids: set[str] = set()
    loop_ids = (
        *((surface.outer_loop_ref,) if surface.outer_loop_ref is not None else ()),
        *surface.hole_loop_refs,
    )
    for loop_id in loop_ids:
        loop = loops_by_id.get(loop_id)
        if loop is None:
            continue
        curve_ids.update(curve_ref.curve_id for curve_ref in loop.curve_refs)
    return {curve_tags[curve_id] for curve_id in curve_ids if curve_id in curve_tags}


def _surface_region(
    geometry_ref: Mapping[str, Any],
    parent_geometry_ref: Mapping[str, Any] | None = None,
) -> tuple[Any, ...]:
    if "outer_loop" in geometry_ref and "quad_points" not in geometry_ref:
        return _planar_region(geometry_ref)
    if "quad_points" in geometry_ref:
        return _quad_region(geometry_ref, parent_geometry_ref or geometry_ref)
    return ()


def _planar_region(geometry_ref: Mapping[str, Any]) -> tuple[Any, ...]:
    import gdstk

    outer = gdstk.Polygon(_clean_loop2d(geometry_ref["outer_loop"]))
    holes = tuple(
        gdstk.Polygon(_clean_loop2d(loop))
        for loop in geometry_ref.get("hole_loops", ())
    )
    if not holes:
        return (outer,)
    return tuple(gdstk.boolean((outer,), holes, "not", precision=_COORD_TOL_UM) or ())


def _quad_region(
    geometry_ref: Mapping[str, Any],
    parent_geometry_ref: Mapping[str, Any],
) -> tuple[Any, ...]:
    import gdstk

    parent = tuple(
        (float(point[0]), float(point[1]), float(point[2]))
        for point in parent_geometry_ref.get("quad_points", ())
    )
    points = tuple(
        (float(point[0]), float(point[1]), float(point[2]))
        for point in geometry_ref.get("quad_points", ())
    )
    if len(parent) != 4 or len(points) != 4:
        return ()
    origin = parent[0]
    u_vec = _vector(parent[0], parent[1])
    v_vec = _vector(parent[0], parent[3])
    u_len2 = _dot(u_vec, u_vec)
    v_len2 = _dot(v_vec, v_vec)
    if u_len2 <= 0 or v_len2 <= 0:
        return ()
    u_len = sqrt(u_len2)
    v_len = sqrt(v_len2)
    uv_points = []
    for point in points:
        rel = _vector(origin, point)
        uv_points.append((_dot(rel, u_vec) / u_len, _dot(rel, v_vec) / v_len))
    return (gdstk.Polygon(uv_points),)


def _boolean_region(
    left: Sequence[Any],
    right: Sequence[Any],
    operation: str,
) -> tuple[Any, ...]:
    import gdstk

    if not left:
        return ()
    if operation == "or":
        return tuple(gdstk.boolean(left, [], "or", precision=_COORD_TOL_UM) or ())
    if not right:
        return tuple(left)
    return tuple(gdstk.boolean(left, right, operation, precision=_COORD_TOL_UM) or ())


def _region_area(region: Sequence[Any]) -> float:
    return sum(abs(float(polygon.area())) for polygon in region)


def _clean_loop2d(loop: Any) -> tuple[tuple[float, float], ...]:
    points = tuple((float(point[0]), float(point[1])) for point in loop)
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    return points


def _vector(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (end[0] - start[0], end[1] - start[1], end[2] - start[2])


def _dot(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _report(
    plan: ConstructionPlanRecord,
    engine_gate: str,
    stage: str,
    failures: list[dict[str, Any]],
    records: list[dict[str, Any]],
    *,
    counts: Mapping[str, int],
) -> dict[str, Any]:
    return {
        "schema": "sgb.engine_gate.v1",
        "engine_gate": engine_gate,
        "route": plan.route,
        "status": "fail" if failures else "pass",
        "stage": stage,
        "checked_record_ids": {
            "interfaces": [record.interface_id for record in plan.interfaces],
            "surface_partitions": [
                record.partition_id for record in plan.surface_partitions
            ],
            "coplanar_inset_families": [
                record.family_id for record in plan.coplanar_inset_families
            ],
            "mesh_size_hints": [
                record.source_partition_id
                for record in plan.mesh_size_hints
                if record.source_partition_id is not None
            ],
            "points": [record.point_id for record in plan.points],
            "curves": [record.curve_id for record in plan.curves],
            "surface_loops": [record.loop_id for record in plan.surface_loops],
            "surfaces": [record.surface_id for record in plan.surfaces],
            "volumes": [record.volume_id for record in plan.volumes],
            "backend_entity_tags": [
                record.source_record_id for record in plan.backend_entity_tags
            ],
        },
        "counts": dict(counts),
        "tolerances": {
            "coordinate_um": _COORD_TOL_UM,
            "area_um2": _AREA_TOL_UM2,
        },
        "failures": failures,
        "records": records,
    }


def _failure(
    code: str,
    record_ids: Sequence[str],
    message: str,
) -> dict[str, Any]:
    return {
        "code": code,
        "record_ids": list(record_ids),
        "message": message,
    }
