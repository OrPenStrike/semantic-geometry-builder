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

from pathlib import Path

from semantic_geometry_builder.models import ConstructionPlanRecord


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
    raise NotImplementedError(
        "canonical PointPlan/CurvePlan/SurfaceLoop OCC lowering is not "
        "implemented yet; raw geometry_ref lowering is disabled"
    )


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
