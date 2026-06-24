# Semantic Geometry Builder

Semantic Geometry Builder is a small, layout-tool-agnostic Python package for
describing semantic CAD topology before solver-specific mesh/config lowering.

It owns the intermediate records and route pipeline between layout-tool
adapters and mesh backends:

```text
GDSFactory / KQCircuits / KLayout adapter
        -> GeometryBuildInput
        -> semantic route geometry pipeline
        -> FinalTopologyRecord + FinalPhysicalGroupRecord
        -> solver-specific adapter such as gsim
```

The package does not load PDKs, depend on GDSFactory or KQCircuits, write
Palace config, build notebooks, or own reports. Those are adapter and consumer
responsibilities.

## Current Scope

This repository currently contains the public IR and fail-fast pipeline
scaffold. Concrete Gmsh/OCC implementation will be added after the record
contracts are reviewed.

## Package Names

- Distribution: `semantic-geometry-builder`
- Import package: `semantic_geometry_builder`
