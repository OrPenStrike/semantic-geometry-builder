"""Gmsh/OpenCASCADE bottom-up construction backend scaffold.

This backend consumes `ConstructionPlanRecord`. It must not discover interfaces
by running global `occ.fragment()` over arbitrary volumes.

Expected implementation shape:

1. create planned points, curves, loops, and live surfaces;
2. create Route A/B construction bodies only for `ConstructionBodyPlanRecord`;
3. execute `CutHostOperationRecord` plans and recover exposed shell surfaces;
4. reuse the same surface tag for planned conformal interfaces;
5. assemble every backend-live volume with `occ.addSurfaceLoop()` and
   `occ.addVolume()`;
6. recover backend dim-tags by `SurfacePlanRecord.surface_id` and
   `VolumePlanRecord.volume_id`;
7. write those tags to `BackendEntityTagRecord`;
8. attach physical groups only from `TagPlanRecord`;
9. write XAO/metadata with the same stable source ids;
10. fail when the plan cannot be built conformally.

Concrete Gmsh/OCC lowering:

- For each unique 2D/3D coordinate in a planned loop, call
  `gmsh.model.occ.addPoint()`.
- For each edge in `outer_loop` or `hole_loops`, call
  `gmsh.model.occ.addLine()`.
- For each closed ring, call `gmsh.model.occ.addCurveLoop()`.
- For each planned surface, call `gmsh.model.occ.addPlaneSurface()` with the
  outer loop and any hole loops in one call.
- For each backend-live volume, call `gmsh.model.occ.addSurfaceLoop()` with the
  planned `SurfaceRefRecord`s, then `gmsh.model.occ.addVolume()`.
- After `gmsh.model.occ.synchronize()`, recover dim-tags by source record id,
  call `gmsh.model.addPhysicalGroup()` for each `TagPlanRecord`, call
  `gmsh.model.setPhysicalName()` with the planned physical name, and write one
  route XAO file.

Do not create volumes directly with boxes, extrusions, or `geometry_ref`
fallbacks. `domain_bounds_um` may help the planner create outer boundary
surfaces, but the backend only accepts volumes whose boundary surfaces already
exist in the plan.

Ground-plane subtraction should be handled as planned surface geometry in the
OCC kernel, not by preprocessing GDS into fragmented positive polygons.
Surface partitions such as inset rings are not backend cuts: the backend should
receive their child `SurfacePlanRecord`s already expanded and build those child
surfaces directly.

OCC backend comment block:

- `SurfacePlanRecord.geometry_ref` must be the direct input for surface
  creation. It should contain a `plane` or `contact_plane`, an `outer_loop`,
  optional `hole_loops`, and optional `loop_geometry_ref`. Missing loop data is
  a build error, not permission to run global fragment discovery.
- Indium-bump or airbridge contact with a ground plane must become one planned
  contact patch surface plus one or more remainder surfaces carrying that patch
  as a hole loop. The contact patch is the shared conformal surface; the hole is
  only how the remainder face avoids overlap.
- Inset rings must arrive as child surface plans with their own loops and
  `inset_band` metadata. Build those child surfaces directly and do not create
  overlay masks or cut a live parent surface after the fact.
- Route A surface sheets are interface surfaces, not extra standalone faces.
  If the plan has an `MS`, `MA`, `MM`, or `SA` surface owned by a sheet
  conductor, build that one live surface and tag it; do not add another sheet on
  top of it.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

from semantic_geometry_builder.models import (
    BackendEntityTagRecord,
    ConstructionPlanRecord,
    GmshDimTag,
    TagPlanRecord,
)


def write_occ_geometry_from_plan(
    plan: ConstructionPlanRecord,
    *,
    xao_path: Path,
) -> ConstructionPlanRecord:
    """Write one XAO and return the plan with backend tags attached.

    The implementation attaches physical groups before writing `xao_path` and
    returns a copy of `plan` with `backend_entity_tags` populated. It refuses
    any volume that lacks planned `surface_refs`, because constructing that
    volume directly would bypass semantic surface ownership.
    """
    import gmsh

    _validate_conformal_plan(plan)
    xao_path.parent.mkdir(parents=True, exist_ok=True)
    was_initialized = bool(gmsh.isInitialized())
    if not was_initialized:
        gmsh.initialize()
    try:
        gmsh.clear()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(f"semantic_geometry_route_{plan.route.lower()}")

        source_tags: dict[tuple[str, str], list[GmshDimTag]] = {}
        for surface in plan.surfaces:
            try:
                surface_tag = _add_surface_from_geometry_ref(
                    gmsh,
                    surface.geometry_ref,
                )
            except Exception as exc:
                raise RuntimeError(
                    f"failed to build surface {surface.surface_id}"
                ) from exc
            source_tags.setdefault(("surface", surface.surface_id), []).extend(
                [(2, surface_tag)]
            )

        for volume in plan.volumes:
            surface_tags = _volume_surface_tags(volume, source_tags)
            if not surface_tags:
                raise ValueError(
                    f"{volume.volume_id} requires planned surface_refs before "
                    "backend volume creation"
                )
            volume_tag = _add_volume_from_surface_tags(gmsh, surface_tags)
            source_tags.setdefault(("volume", volume.volume_id), []).append(
                (3, volume_tag)
            )

        gmsh.model.occ.synchronize()

        backend_tags = tuple(
            BackendEntityTagRecord(
                source_record_kind=source_kind,
                source_record_id=source_id,
                dim_tag=dim_tag,
            )
            for (source_kind, source_id), dim_tags in source_tags.items()
            for dim_tag in dim_tags
        )

        for tag_plans in _group_tag_plans(plan.tags):
            first_tag = tag_plans[0]
            entity_tags = _physical_entity_tags(tag_plans, source_tags)
            if not entity_tags:
                raise ValueError(
                    f"{first_tag.physical_name} has no backend entity tags"
                )
            group_tag = gmsh.model.addPhysicalGroup(
                first_tag.dimension,
                entity_tags,
            )
            gmsh.model.setPhysicalName(
                first_tag.dimension,
                group_tag,
                first_tag.physical_name,
            )

        gmsh.write(str(xao_path))
        return replace(
            plan,
            backend_entity_tags=backend_tags,
            metadata={
                **dict(plan.metadata),
                "xao_path": str(xao_path),
            },
        )
    finally:
        if not was_initialized:
            gmsh.finalize()


def _validate_conformal_plan(plan: ConstructionPlanRecord) -> None:
    """Reject any backend-live volume that would need direct construction."""
    unpartitioned_volumes = [
        volume.volume_id
        for volume in plan.volumes
        if not volume.construction_only and not volume.surface_refs
    ]
    if unpartitioned_volumes:
        raise NotImplementedError(
            "nonconformal OCC build refused: backend-live volumes must be "
            "assembled from planned surface_refs with addSurfaceLoop/addVolume "
            f"{unpartitioned_volumes!r}. Building these directly would create "
            "standalone surfaces instead of shared conformal topology."
        )


def _volume_surface_tags(
    volume: Any,
    source_tags: Mapping[tuple[str, str], Sequence[GmshDimTag]],
) -> tuple[int, ...]:
    return tuple(
        tag
        for surface_ref in volume.surface_refs
        for dim, tag in source_tags.get(("surface", surface_ref.surface_id), ())
        if dim == 2
    )


def _group_tag_plans(
    tags: tuple[TagPlanRecord, ...],
) -> tuple[tuple[TagPlanRecord, ...], ...]:
    grouped: dict[tuple[str, int, str, str], list[TagPlanRecord]] = {}
    for tag in tags:
        grouped.setdefault(
            (tag.physical_name, tag.dimension, tag.role, tag.solver_use),
            [],
        ).append(tag)
    return tuple(tuple(items) for items in grouped.values())


def _physical_entity_tags(
    tag_plans: tuple[TagPlanRecord, ...],
    source_tags: Mapping[tuple[str, str], Sequence[GmshDimTag]],
) -> list[int]:
    entity_tags: list[int] = []
    seen: set[int] = set()
    for tag_plan in tag_plans:
        for dimension, entity_tag in source_tags.get(
            (tag_plan.source_record_kind, tag_plan.source_record_id),
            (),
        ):
            if dimension != tag_plan.dimension or entity_tag in seen:
                continue
            entity_tags.append(entity_tag)
            seen.add(entity_tag)
    return entity_tags


def _add_volume_from_surface_tags(gmsh: Any, surface_tags: Sequence[int]) -> int:
    shell_tag = gmsh.model.occ.addSurfaceLoop(list(surface_tags), sewing=True)
    return gmsh.model.occ.addVolume([shell_tag])


def _add_surface_from_geometry_ref(
    gmsh: Any,
    geometry_ref: Mapping[str, Any],
) -> int:
    if "quad_points" in geometry_ref:
        return _add_quad_points_surface(gmsh, geometry_ref["quad_points"])
    if geometry_ref.get("shell_part") == "sidewall":
        raise ValueError("sidewall surfaces must be split into quad_points")
    if "shell_part" in geometry_ref:
        return _add_shell_from_geometry_ref(gmsh, geometry_ref)
    z_um = _plane_z_um(geometry_ref)
    return _add_plane_surface(
        gmsh,
        _outer_loop(geometry_ref),
        _hole_loops(geometry_ref),
        z_um,
    )


def _add_shell_from_geometry_ref(
    gmsh: Any,
    geometry_ref: Mapping[str, Any],
) -> int:
    outer_loop = _outer_loop(geometry_ref)
    hole_loops = _hole_loops(geometry_ref)
    z_min_um = float(geometry_ref.get("z_min_um", geometry_ref.get("z_um", 0.0)))
    thickness_um = float(geometry_ref.get("thickness_um", 0.0))
    shell_part = geometry_ref.get("shell_part")
    if thickness_um <= 0:
        return _add_plane_surface(gmsh, outer_loop, hole_loops, z_min_um)
    z_max_um = z_min_um + thickness_um
    if shell_part == "top":
        return _add_plane_surface(gmsh, outer_loop, hole_loops, z_max_um)
    if shell_part == "bottom":
        return _add_plane_surface(gmsh, outer_loop, hole_loops, z_min_um)
    raise ValueError(f"unsupported shell_part {shell_part!r}")


def _add_plane_surface(
    gmsh: Any,
    outer_loop: Sequence[Sequence[float]],
    hole_loops: Sequence[Sequence[Sequence[float]]],
    z_um: float,
) -> int:
    loop_tags = [
        _add_curve_loop(gmsh, ((x, y, z_um) for x, y in _clean_ring(outer_loop)))
    ]
    loop_tags.extend(
        _add_curve_loop(gmsh, ((x, y, z_um) for x, y in _clean_ring(hole_loop)))
        for hole_loop in hole_loops
    )
    return gmsh.model.occ.addPlaneSurface(loop_tags)


def _add_quad_points_surface(
    gmsh: Any,
    points: Sequence[Sequence[float]],
) -> int:
    if len(points) != 4:
        raise ValueError("quad_points requires exactly 4 points")
    loop_tag = _add_curve_loop(
        gmsh,
        tuple((float(point[0]), float(point[1]), float(point[2])) for point in points),
    )
    return gmsh.model.occ.addPlaneSurface([loop_tag])


def _add_curve_loop(
    gmsh: Any,
    points: Sequence[tuple[float, float, float]] | Any,
) -> int:
    clean_points = tuple(points)
    if len(clean_points) < 3:
        raise ValueError("curve loop requires at least 3 points")
    point_tags = [
        gmsh.model.occ.addPoint(float(x), float(y), float(z))
        for x, y, z in clean_points
    ]
    line_tags = [
        gmsh.model.occ.addLine(
            point_tags[index],
            point_tags[(index + 1) % len(point_tags)],
        )
        for index in range(len(point_tags))
    ]
    return gmsh.model.occ.addCurveLoop(line_tags)


def _plane_z_um(geometry_ref: Mapping[str, Any]) -> float:
    plane = geometry_ref.get("plane") or geometry_ref.get("contact_plane")
    if isinstance(plane, Mapping) and plane.get("axis") == "z":
        return float(plane["value_um"])
    return float(geometry_ref.get("z_um", 0.0))


def _outer_loop(geometry_ref: Mapping[str, Any]) -> tuple[tuple[float, float], ...]:
    if "outer_loop" not in geometry_ref:
        raise ValueError("geometry_ref requires outer_loop")
    return _clean_ring(geometry_ref["outer_loop"])


def _hole_loops(
    geometry_ref: Mapping[str, Any],
) -> tuple[tuple[tuple[float, float], ...], ...]:
    return tuple(
        _clean_ring(hole_loop)
        for hole_loop in geometry_ref.get("hole_loops", ())
    )


def _clean_ring(ring: Sequence[Sequence[float]]) -> tuple[tuple[float, float], ...]:
    points = tuple((float(point[0]), float(point[1])) for point in ring)
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("ring requires at least 3 unique points")
    return points


def _ring_edges(
    ring: Sequence[tuple[float, float]],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    return tuple(
        (ring[index], ring[(index + 1) % len(ring)])
        for index in range(len(ring))
    )
