"""Gmsh/OpenCASCADE bottom-up construction backend scaffold.

This backend consumes `ConstructionPlanRecord`. It must not discover interfaces
by running global `occ.fragment()` over arbitrary volumes.

Expected implementation shape:

1. create planned points, curves, loops, and live surfaces;
2. create Route A/B construction bodies only for `ConstructionBodyPlanRecord`;
3. execute `CutHostOperationRecord` plans and recover exposed shell surfaces;
4. reuse the same surface tag for planned conformal interfaces;
5. assemble each Route C volume with `occ.addSurfaceLoop()` and
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
- For each retained Route C volume, call `gmsh.model.occ.addSurfaceLoop()` with
  the planned `SurfaceRefRecord`s, then `gmsh.model.occ.addVolume()`.
- After `gmsh.model.occ.synchronize()`, recover dim-tags by source record id,
  call `gmsh.model.addPhysicalGroup()` for each `TagPlanRecord`, call
  `gmsh.model.setPhysicalName()` with the planned physical name, and write one
  route XAO file.

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

from pathlib import Path

from semantic_geometry_builder.models import ConstructionPlanRecord


def write_occ_geometry_from_plan(
    plan: ConstructionPlanRecord,
    *,
    xao_path: Path,
) -> ConstructionPlanRecord:
    """Write one XAO and return the plan with backend tags attached.

    The implementation must attach physical groups before writing `xao_path`.
    It should return a copy of `plan` with `backend_entity_tags` populated so
    `export_physical_group_records()` can produce the review sidecar.
    """
    del plan, xao_path
    raise NotImplementedError(
        "bottom-up OCC writer is not implemented; global fragment-first backend "
        "has been removed"
    )
