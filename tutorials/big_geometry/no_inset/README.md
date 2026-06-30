# Big Geometry - No Inset

Use these notebooks for the current `sim_flip_chip_distance` Route B probe:

- [build ignore-port-sheet XAO](sim_flip_chip_distance_route_b_build_ignore_port_sheet_probe.ipynb)
- [mesh profile: no field](sim_flip_chip_distance_route_b_mesh_no_field_profile.ipynb)
- [mesh profile: Distance field](sim_flip_chip_distance_route_b_mesh_distance_field_profile.ipynb)

The original big-geometry no-inset route notebooks remain at:

- [route_a](../../no_inset/sim_flip_chip_distance_route_a.ipynb)
- [route_b](../../no_inset/sim_flip_chip_distance_route_b.ipynb)
- [route_c](../../no_inset/sim_flip_chip_distance_route_c.ipynb)

Observed artifacts live under:

```text
tutorials/runs/no_inset/sim_flip_chip_distance/route_b_ignore_port_sheet_probe/
```

Current measured result:

| Profile | Stage | Time | Result |
| --- | ---: | ---: | --- |
| no field | `generate(1)` | `2.89s` | `154,287` 1D elements |
| no field | `generate(2)` | `119.47s` | `4,346,569` 2D elements |
| no field | `generate(3)` | `>5 min` | interrupted |
| Distance field | setup | `0.20s` | `154,019` boundary curves |
| Distance field | `generate(1)` | `>5 min` | interrupted |
