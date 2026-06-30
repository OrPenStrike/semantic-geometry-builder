# Tutorials

Tutorial notebooks are grouped by geometry size first, then by inset policy.
The older `no_inset/` and `with_inset/` folders remain as stable source
locations for the original route notebooks.

## Small Geometry

Small geometry notebooks are expected to build quickly enough for routine route
checks.

- [small_geometry/no_inset](small_geometry/no_inset/README.md)
- [small_geometry/with_inset](small_geometry/with_inset/README.md)

Current small fixtures:

- `martinis2022_ribbon`
- `resonator`
- `resonator_with_indium_bumps`

## Big Geometry

Big geometry notebooks keep build and mesh profiling separate. Mesh profiling
starts with the no-field Gmsh baseline before enabling Distance-field
refinement.

- [big_geometry/no_inset](big_geometry/no_inset/README.md)
- [big_geometry/with_inset](big_geometry/with_inset/README.md)

Current big fixture:

- `sim_flip_chip_distance`
