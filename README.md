# Semantic Geometry Builder

Semantic Geometry Builder is the standalone source of truth for semantic CAD
topology before solver-specific mesh/config lowering. Layout frontends provide
the same normalized `GeometryBuildInput`; this package owns route planning,
semantic topology records, and final physical-group plans.

```text
GDSFactory / OrPen-SC-PDK / KQCircuits / KLayout tech-file adapter
        -> GeometryBuildInput
        -> Semantic Geometry Builder
        -> FinalTopologyRecord + FinalPhysicalGroupRecord + audit artifacts
        -> solver-specific adapter
        -> mesh/config/reporting
```

Semantic identity is stable. Temporary backend handles such as Gmsh/OCC tags
may appear in provenance fields, but they must not become the source of truth.
Stable identity lives in semantic ids, interface ids, ring ids, operation ids,
physical names, and metadata/audit records.

## Ownership

This package owns:

- `GeometryBuildInput` as the layout-tool-agnostic input IR.
- `SemanticEntitySpec` as the stable semantic source of truth.
- `solution_regions` as construction metadata keyed by solution-domain
  semantic ids such as `AIR`, substrate, or dielectric regions.
- Primitive, reference-topology, interface, inset-ring, route-materialization,
  final-topology, and final physical-group records.
- Audit/provenance contracts for future geometry snapshots.

This package does not own:

- PDK or layer-stack loading.
- GDSFactory, KQCircuits, or KLayout object APIs.
- Solver config generation, run-folder policy, reports, notebooks, or result
  plotting.
- Solver-specific postprocessing semantics.

## Current Scope

This repository currently contains the public API, data contracts, and
fail-fast pipeline scaffold. Concrete Gmsh/OCC implementation comes after the
record contracts are reviewed.

Current source layers:

- `src/semantic_geometry_builder/__init__.py`: public package API for frontend
  adapters and solver consumers.
- `src/semantic_geometry_builder/adapter.py`: frontend adapter contracts
  that build `GeometryBuildInput` from tool-specific technology objects,
  including GDS files plus project stackup/tech files such as
  Ansys/HFSS/Q3D-style dialects.
- `src/semantic_geometry_builder/pipeline.py`: semantic route pipeline facade,
  validation, and stage functions for already-normalized input records.
- `src/semantic_geometry_builder/route_materialization/`: Route A/B/C
  materialization sub-pipeline; `pipeline.materialize_route()` is only the
  dispatcher and common validation boundary.
- `src/semantic_geometry_builder/models.py`: layout-tool-agnostic records,
  type aliases, and small contract guards.

## Run Folder Contract

The builder is run-folder aware. `SemanticGeometryBuilder.build(...)` receives a
`run_folder` path and owns semantic-geometry metadata sidecars under:

```text
<run_folder>/
    geometry/
    logs/
    metadata/
        semantic_geometry/
            01_validate_geometry_input.json
            02_build_semantic_primitives.json
            ...
    results/
```

The `metadata/semantic_geometry/` files are review/audit artifacts for this
package's pipeline stages. They may reference optional geometry snapshots, but
they must not become Palace config, mesh generation output, or solver result
files. Downstream consumers can read these sidecars when building manifests,
assigning backend physical groups, or generating solver config.

## Route Contract

- Route A: mixed surface-sheet / PEC-shell representation. Face metals and
  attached airbridge decks become solver-active `surface_sheet` conductors.
  Indium bumps and airbridge posts use construction bodies to cut host regions,
  then expose solver-facing PEC `cutout_boundary_shell` surfaces.
- Route B: cut-out PEC boundary-shell representation. All conductors use
  construction bodies to remove conductor interiors from solution regions, and
  exposed PEC `cutout_boundary_shell` surfaces become the solver-facing
  conductor representation.
- Route C: retained material-volume representation. Conductors remain as
  `material_volume` regions, and interfaces come from retained volume adjacency
  and contact partitioning.

Use `AIR` as the semantic domain name for air or vacuum-like solution regions.
Solver adapters can map `AIR` to their physical material vocabulary.

## Interface And Inset Contract

`MS`, `MA`, and `SA` are solver-active interface kinds by default. `MM`, `SS`,
and `AA` are valid topology/contact/audit interfaces, but callers should mark
them `audit_only` or `postprocessing_only` unless a solver adapter explicitly
uses them.

`plan_inset_rings()` only plans ring/core intent such as `BAND_0_50NM`,
`BAND_50NM_100NM`, or `CORE_AFTER_1UM`. It must not create backend topology.
`final_boolean_topology_build()` is the stage that turns surviving ring plans
into conformal final topology. Inset rings must partition the parent interface:
child ring/core surfaces replace the parent as live geometry, and the parent
surface becomes a logical aggregate only. Overlay ring surfaces are not a
geometry or mesh mode, and overlay masks are intentionally unsupported because
they can create nonconformal geometry, duplicate parent/child boundary
ownership, and ambiguous solver/EPR surface integration.

## Physical Groups

`FinalPhysicalGroupRecord` describes a solver-neutral physical-group plan.
Logical aggregates such as `TOTAL`, `TOTAL_MA`, or `IF__...__TOTAL` are not
real physical groups. They must use `logical_only=True`, carry no backend entity
tags, and point to child physical names instead.

## Package Names

- Distribution: `semantic-geometry-builder`
- Import package: `semantic_geometry_builder`
