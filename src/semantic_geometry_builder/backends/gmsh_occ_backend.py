"""Gmsh/OpenCASCADE bottom-up construction backend scaffold.

This backend consumes `ConstructionPlanRecord`. It must not discover interfaces
by running global `occ.fragment()` over arbitrary volumes.

Expected implementation shape:

1. consume planned canonical curves/surface loops and create live surfaces;
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

Concrete Gmsh/OCC lowering target:

- For each unique point referenced by `CurvePlanRecord`, call
  `gmsh.model.occ.addPoint()`.
- For each `CurvePlanRecord`, call
  `gmsh.model.occ.addLine()`.
- For each `SurfaceLoopRecord`, call `gmsh.model.occ.addCurveLoop()` using
  planned `CurveRefRecord` order. Do not rely on negative OCC curve tags as the
  only correctness mechanism; loop closure and normals must already be
  validated by the compiler plan and audited after lowering.
- For each planned surface, call `gmsh.model.occ.addPlaneSurface()` with the
  planned outer loop and any hole loops in one call.
- For each backend-live volume, call `gmsh.model.occ.addSurfaceLoop()` with the
  planned `SurfaceRefRecord`s, then `gmsh.model.occ.addVolume()`.
- After `gmsh.model.occ.synchronize()`, recover dim-tags by source record id,
  call `gmsh.model.addPhysicalGroup()` for each `TagPlanRecord`, call
  `gmsh.model.setPhysicalName()` with the planned physical name, and write one
  route XAO file.
- After lowering, audit `getBoundary()` results for surfaces and volumes
  against planned curves/surfaces before treating the XAO as solver-ready.

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

- `SurfacePlanRecord.outer_loop_ref` and `hole_loop_refs` are the v1 backend
  contract. `geometry_ref` is retained as audit/source metadata only. Raw-loop
  lowering is intentionally disabled until canonical point/curve/surface-loop
  lowering is implemented.
- `TagPlanRecord` plus `BackendEntityTagRecord` is the tag ledger. Plan ids are
  stable; OCC tags and physical group ids are lowering results. If two live
  source ids map to one OCC dim-tag, canonicalization failed and the backend
  must report it instead of merging identities.
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
    SurfaceLoopRecord,
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
    _validate_conformal_plan(plan)
    import gmsh

    xao_path.parent.mkdir(parents=True, exist_ok=True)
    was_initialized = bool(gmsh.isInitialized())
    if not was_initialized:
        gmsh.initialize()
    try:
        gmsh.clear()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(f"semantic_geometry_route_{plan.route.lower()}")

        point_tags = {
            point.point_id: gmsh.model.occ.addPoint(*point.coordinate)
            for point in plan.points
        }
        curve_tags = {
            curve.curve_id: gmsh.model.occ.addLine(
                point_tags[curve.start_point_id],
                point_tags[curve.end_point_id],
            )
            for curve in plan.curves
        }
        loop_tags = {
            loop.loop_id: _add_curve_loop_from_plan(gmsh, loop, curve_tags)
            for loop in plan.surface_loops
        }
        source_tags: dict[tuple[str, str], list[GmshDimTag]] = {}
        for surface in plan.surfaces:
            if surface.construction_only:
                continue
            surface_tag = gmsh.model.occ.addPlaneSurface(
                [
                    loop_tags[surface.outer_loop_ref],
                    *(loop_tags[loop_id] for loop_id in surface.hole_loop_refs),
                ]
            )
            source_tags.setdefault(("surface", surface.surface_id), []).append(
                (2, surface_tag)
            )

        for volume in plan.volumes:
            if volume.construction_only:
                continue
            surface_tags = _volume_surface_tags(volume, source_tags)
            shell_tag = gmsh.model.occ.addSurfaceLoop(surface_tags)
            volume_tag = gmsh.model.occ.addVolume([shell_tag])
            source_tags.setdefault(("volume", volume.volume_id), []).append(
                (3, volume_tag)
            )

        gmsh.model.occ.synchronize()
        backend_tags = _backend_entity_tags(source_tags)
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
            metadata={**dict(plan.metadata), "xao_path": str(xao_path)},
        )
    finally:
        if not was_initialized:
            gmsh.finalize()


def _validate_conformal_plan(plan: ConstructionPlanRecord) -> None:
    """Reject any backend-live geometry that lacks canonical topology refs."""
    if any(not surface.construction_only for surface in plan.surfaces) and (
        not plan.points or not plan.curves or not plan.surface_loops
    ):
        raise NotImplementedError(
            "nonconformal OCC build refused: backend-live surfaces require "
            "planned PointPlan/CurvePlan/SurfaceLoop records before lowering"
        )
    missing_loop_refs = [
        surface.surface_id
        for surface in plan.surfaces
        if not surface.construction_only and surface.outer_loop_ref is None
    ]
    if missing_loop_refs:
        raise NotImplementedError(
            "nonconformal OCC build refused: backend-live surfaces require "
            f"outer_loop_ref before lowering {missing_loop_refs!r}"
        )
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


def _add_curve_loop_from_plan(
    gmsh: Any,
    loop: SurfaceLoopRecord,
    curve_tags: Mapping[str, int],
) -> int:
    signed_curve_tags = [
        curve_tags[curve_ref.curve_id] * curve_ref.orientation
        for curve_ref in loop.curve_refs
    ]
    return gmsh.model.occ.addCurveLoop(signed_curve_tags)


def _volume_surface_tags(
    volume: Any,
    source_tags: Mapping[tuple[str, str], Sequence[GmshDimTag]],
) -> list[int]:
    tags = [
        tag
        for surface_ref in volume.surface_refs
        for dim, tag in source_tags.get(("surface", surface_ref.surface_id), ())
        if dim == 2
    ]
    if len(tags) != len(volume.surface_refs):
        missing = [
            surface_ref.surface_id
            for surface_ref in volume.surface_refs
            if not source_tags.get(("surface", surface_ref.surface_id), ())
        ]
        raise ValueError(f"{volume.volume_id} missing surface tags: {missing!r}")
    return tags


def _backend_entity_tags(
    source_tags: Mapping[tuple[str, str], Sequence[GmshDimTag]],
) -> tuple[BackendEntityTagRecord, ...]:
    return tuple(
        BackendEntityTagRecord(
            source_record_kind=source_kind,
            source_record_id=source_id,
            dim_tag=dim_tag,
        )
        for (source_kind, source_id), dim_tags in source_tags.items()
        for dim_tag in dim_tags
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
    tags: list[int] = []
    seen: set[int] = set()
    for tag_plan in tag_plans:
        for dimension, tag in source_tags.get(
            (tag_plan.source_record_kind, tag_plan.source_record_id),
            (),
        ):
            if dimension != tag_plan.dimension or tag in seen:
                continue
            tags.append(tag)
            seen.add(tag)
    return tags
