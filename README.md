# Semantic Geometry Builder

Semantic Geometry Builder is the standalone source of truth for semantic CAD
topology before solver-specific mesh/config lowering. Layout frontends provide
the same normalized `GeometryBuildInput`; this package owns route planning,
semantic topology records, and final physical-group plans.

```text
GDS + semantic stack JSON adapter (Level 0)
        -> GeometryBuildInput
        -> optional GDSFactory / gsim / KQCircuits lowering through Level 0
        -> Semantic Geometry Builder
        -> route XAO + FinalPhysicalGroupRecord + audit artifacts
        -> solver-specific adapter
        -> mesh/config/reporting
```

Semantic identity is stable. Temporary backend handles such as Gmsh/OCC tags
may appear in provenance fields, but they must not become the source of truth.
Stable identity lives in semantic ids, interface ids, surface ids, volume ids,
physical names, and metadata/audit records.

## Review Path

Read the package in this order:

1. Folder structure: `adapter.py` lowers frontend files into
   `GeometryBuildInput`; `models.py` defines the records; `planning.py` turns
   records into a route-specific construction plan; `backends/` builds the OCC
   model; `export.py` projects backend tags back to physical groups.
2. Models: `InterfacePlanRecord`, `SurfacePartitionRecord`,
   `SurfacePlanRecord`, `VolumePlanRecord`, `ConstructionBodyPlanRecord`,
   `CutHostOperationRecord`, and `TagPlanRecord` are the metadata ladder from
   semantic layout intent to physical groups.
3. Pipeline: `SemanticGeometryBuilder.build()` is the top-level flow. It writes
   stage sidecars, calls `build_route_construction_plan()`, calls the OCC
   backend, then exports `FinalPhysicalGroupRecord`s.
4. Routes: Route A produces interface-owned sheets plus PEC shells; Route B
   produces cutout shells; Route C produces retained material volumes.
5. Backend: planned curves and surface loops become Gmsh/OCC points, lines,
   curve loops, plane surfaces, surface loops, volumes, physical groups, and
   finally one XAO file.
6. Output: one build writes one route XAO file under `geometry/` plus JSON
   sidecars under `metadata/semantic_geometry/`. This package does not write a
   mesh file.

## Ownership

This package owns:

- `GeometryBuildInput` as the layout-tool-agnostic input IR.
- `SemanticEntitySpec` as the stable semantic source of truth.
- `solution_regions` as construction metadata keyed by solution-domain
  semantic ids such as `AIR`, substrate, or dielectric regions.
- Pre-construction interface plans, surface partition plans, live surface plans,
  volume plans, tag plans, and final physical-group records.
- Audit/provenance contracts for future geometry snapshots.

This package does not own:

- PDK or layer-stack loading beyond the reviewed Level 0 stack JSON contract.
- Native GDSFactory, gsim, KQCircuits, or KLayout object APIs.
- Solver config generation, run-folder policy, reports, notebooks, or result
  plotting.
- Solver-specific postprocessing semantics.

## Current Scope

This repository currently contains the public API, data contracts, adapter
input lowering, and an actively reviewed geometry-backend scaffold. The backend
direction is being reset from fragment-first construction to
surface-plan-first construction.

The fragment-first approach was tested and rejected for v1 as the default
backend strategy. It is simple and general for small examples, but it pushes a
layered ECAD problem into full 3D BRep boolean fragmentation. On larger
geometries, such as ground planes with high-vertex cutout boundaries, the
workflow becomes too expensive and also makes interface identity depend on
backend boolean results. That is the wrong source of truth for this package.

The v1 backend target is:

```text
GeometryBuildInput
    -> 2D semantic normalization / planar arrangement / stack z-sweep
    -> InterfacePlan
    -> surface candidates / partitions
    -> PointPlan + CurvePlan + SurfaceLoop refs
    -> canonical SurfacePlan
    -> VolumePlan
    -> TagPlan
    -> bottom-up Gmsh/OCC construction
    -> XAO / physical-group export
    -> topology audit / meshability audit
```

The stack JSON is not only a height table. It is the material-occupancy input
that lets the planner sweep normalized 2D cells through z-events and decide
which material/domain owns each 3D cell. Interfaces must then be identified
from cell adjacency before backend geometry is created. Horizontal adjacency
creates top/bottom faces such as `MS`, `MA`, or `SA`; vertical adjacency from
shared atomic 2D edges creates sidewall faces.

Inset rings are planned before OCC construction: the planner should expand the
parent interface into child ring/core `SurfacePlanRecord`s, disable the parent
as live geometry, and only pass non-overlapping children downstream. The
planner also owns the canonical topology registry: shared vertices, shared
edges, and shared face patches must become shared `PointPlanRecord`s,
`CurvePlanRecord`s, and ordered `SurfaceLoopRecord`s before OCC lowering.
Backend line caches are allowed as a lowering optimization, but they are not
the source of truth for conformal topology.

`occ.fragment()` is not the interface discovery engine for v1. It may remain a
local fallback only after the surface/volume plan says a small, specific
partition is required. It must not be used as a global all-to-all construction
strategy, and it must not be the mechanism that assigns semantic identity.
`sewing=True` and `removeAllDuplicates()` are also outside the v1 correctness
path; they hide missing canonical topology instead of proving it.

Validation is split in two. Pre-lowering validation checks canonical points,
curves, loops, surfaces, interface coverage, volume closure, surface use
counts, and tag references without asking Gmsh to repair anything.
Post-lowering audit checks that OCC boundaries and physical groups match the
planned ids. Meshability may be checked in memory as an audit gate, but this
package still does not write `.msh` files.

Current source layers:

- `src/semantic_geometry_builder/__init__.py`: public package API for frontend
  adapters and solver consumers.
- `src/semantic_geometry_builder/adapter.py`: frontend adapter contracts.
  `build_gds_stack_geometry_input()` is the Level 0 supported path: GDS file
  plus semantic `.stack.json`. GDSFactory, gsim, KQCircuits, and other adapters
  should lower their native objects into this same stack semantics.
- `src/semantic_geometry_builder/pipeline.py`: small public facade for
  run-folder orchestration and stable imports.
- `src/semantic_geometry_builder/planning.py`: route-first interface,
  surface-partition, canonical point/curve/surface-loop, construction-body,
  volume, cut-operation, and tag plans.
- `src/semantic_geometry_builder/validation.py`: fail-fast input and plan
  invariants.
- `src/semantic_geometry_builder/export.py`: backend dim-tag to physical-group
  record conversion.
- `src/semantic_geometry_builder/backends/`: backend construction experiments.
  The accepted v1 direction is bottom-up planned surface/volume construction,
  not fragment-first interface discovery.
- `src/semantic_geometry_builder/models.py`: layout-tool-agnostic records,
  type aliases, and small contract guards.

## Run Folder Contract

The builder is run-folder aware. `SemanticGeometryBuilder.build(...)` receives a
`run_folder` path and owns semantic-geometry metadata sidecars under:

```text
<run_folder>/
    geometry/
        semantic_geometry_route_<route>.xao
    logs/
    metadata/
        semantic_geometry/
            01_validate_geometry_input.json
            02_build_route_construction_plan.json
            03_build_occ_geometry.json
            04_export_physical_groups.json
    results/
```

The `metadata/semantic_geometry/` files are review/audit artifacts for this
package's pipeline stages. The stage names should follow the accepted v1 flow:
input validation, route-aware interface/surface-partition/construction-body/
cut-operation/surface/volume/tag planning, backend construction, and
physical-group export. They may reference optional geometry snapshots, but they
must not become Palace config, mesh generation output, or solver result files.
Downstream consumers can read these sidecars when building manifests, assigning
backend physical groups, or generating solver config.

The geometry artifact for one `build()` call is the route XAO file. Physical
groups must be attached before that XAO is written, using the physical names
planned by `TagPlanRecord` and the dim-tags recovered by
`BackendEntityTagRecord`.

## Route Contract

- Route A: mixed surface-sheet / PEC-shell representation. Face metals and
  attached airbridge decks are represented by solver-active `MS`, `MA`, `MM`,
  or `SA` interface surfaces carrying `surface_sheet` semantics.
  Indium bumps and airbridge posts use construction bodies to cut host regions,
  then expose solver-facing PEC `cutout_boundary_shell` surfaces. Route A
  `surface_sheet` entities must not become construction cutters or standalone
  overlay surfaces.
- Route B: cut-out PEC boundary-shell representation. All conductors use
  construction bodies to remove conductor interiors from solution regions, and
  exposed PEC `cutout_boundary_shell` surfaces become the solver-facing
  conductor representation.
- Route C: retained material-volume representation. Conductors remain as
  `material_volume` regions. Required interfaces must be planned before
  backend geometry creation; they must not be discovered by global retained
  volume fragmentation.

Use `AIR` as the semantic domain name for air or vacuum-like solution regions.
Solver adapters can map `AIR` to their physical material vocabulary.

## Interface And Inset Contract

`MM`, `SS`, `AA`, `MS`, `MA`, and `SA` are all solver-active geometry interface
kinds in this package. Solver adapters may choose how to lower each kind, but
the geometry builder should plan them as real interface surfaces when they are
part of the selected route.

Interface ids start directly with the interface kind, for example
`MM__Ground__Resonator__EDGE_0001` or `MS__Metal__Substrate__TOP_0001`.
Do not add an `IF__` prefix.

Inset rings are surface partition intent such as `BAND_0_50NM`,
`BAND_50NM_100NM`, or `CORE_AFTER_1UM`; planning them must not run backend
booleans. A `SurfacePartitionRecord` points from a parent interface to a child
live `SurfacePlanRecord`. The child ring/core surfaces replace the parent as
live geometry, and the parent surface becomes a logical aggregate only. Overlay
ring surfaces are not a geometry or mesh mode, and overlay masks are
intentionally unsupported because they can create nonconformal geometry,
duplicate parent/child boundary ownership, and ambiguous solver/EPR surface
integration.

## Physical Groups

`TagPlanRecord` is created before backend geometry. It points to a live
`SurfacePlanRecord` or `VolumePlanRecord`; after the backend returns dim-tags,
`FinalPhysicalGroupRecord` uses the same source id to create solver-neutral
physical groups.

Backend dim-tags are recorded through `BackendEntityTagRecord`, not by storing
loose tag lists inside tag metadata.

## Fixture Contract

Fixture layouts under `fixtures/open_pdk/` use GDS plus semantic `.stack.json`.
Ansys/HFSS `.tech` files are not the semantic input contract for this package.

## Package Names

- Distribution: `semantic-geometry-builder`
- Import package: `semantic_geometry_builder`
