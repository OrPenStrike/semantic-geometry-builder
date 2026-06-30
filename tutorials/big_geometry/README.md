# Big Geometry

Big geometry notebooks separate three concerns:

1. build or locate an XAO;
2. run no-field Gmsh mesh profiling first;
3. compare against Distance-field refinement only as a diagnostic profile.

The current big fixture is `sim_flip_chip_distance`. Its Palace lumped-port
sheet source layer is still backend fail-fast in the main SGB build path, so
the current no-inset probe intentionally ignores that source layer to isolate
large-XAO mesh behavior.
