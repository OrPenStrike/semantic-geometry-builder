import json

import gdstk

from semantic_geometry_builder import SemanticGeometryBuilder
from semantic_geometry_builder.adapter import build_gds_stack_geometry_input
from semantic_geometry_builder.models import (
    ConstructionPlanRecord,
    GeometryBuildInput,
    LayoutPolygonSpec,
    PortSheetOverlapRecord,
    PortSheetRegionRecord,
    SemanticEntitySpec,
)
from semantic_geometry_builder.validation import validate_geometry_input


def test_adapter_extracts_palace_lumped_port_sheet_region(tmp_path) -> None:
    library = gdstk.Library()
    cell = library.new_cell("TOP")
    cell.add(gdstk.Polygon([(0, 0), (10, 0), (10, 10), (0, 10)], layer=1, datatype=0))
    cell.add(gdstk.Polygon([(2, 2), (8, 2), (8, 8), (2, 8)], layer=202, datatype=1))
    gds_path = tmp_path / "fixture.gds"
    library.write_gds(gds_path)

    stack_path = tmp_path / "fixture.stack.json"
    stack_path.write_text(
        json.dumps(
            {
                "metadata": {
                    "top_cell_name": "TOP",
                    "port_sheet_source_layers": [
                        {
                            "layer": 202,
                            "datatype": 1,
                            "name": "P1",
                            "source": "palace_lumped_port_sheet",
                        }
                    ],
                },
                "solution_regions": {
                    "AIR": {
                        "role": "solution_region",
                        "material_id": "AIR",
                        "geometry": {
                            "z_min_um": 0.0,
                            "z_max_um": 10.0,
                            "domain_bounds_um": {
                                "x_min_um": -1.0,
                                "y_min_um": -1.0,
                                "x_max_um": 11.0,
                                "y_max_um": 11.0,
                            },
                        },
                    }
                },
                "layers": [
                    {
                        "layer": 1,
                        "datatype": 0,
                        "semantic_id": "HOST_METAL",
                        "role": "metal",
                        "material_id": "aluminum",
                        "geometry": {
                            "geometry_source": "gds_polygon",
                            "z_um": 0.0,
                            "thickness_um": 0.2,
                        },
                        "route_representations": {
                            "A": "surface_sheet",
                            "B": "cutout_boundary_shell",
                            "C": "material_volume",
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    build_input = build_gds_stack_geometry_input(
        gds_file=gds_path,
        stack_file=stack_path,
    )

    assert len(build_input.port_sheet_regions) == 1
    region = build_input.port_sheet_regions[0]
    assert region.source_layer == "202/1"
    assert region.metadata["source"] == "palace_lumped_port_sheet"
    assert len(region.overlaps) == 1
    assert region.overlaps[0].host_semantic_id == "HOST_METAL"
    assert all(
        region.source_polygon_id not in entity.polygon_ids
        for entity in build_input.entities
    )
    validate_geometry_input(build_input)


def test_ignored_layer_metadata_does_not_create_port_sheet_region(tmp_path) -> None:
    library = gdstk.Library()
    cell = library.new_cell("TOP")
    cell.add(gdstk.Polygon([(0, 0), (10, 0), (10, 10), (0, 10)], layer=1, datatype=0))
    cell.add(gdstk.Polygon([(2, 2), (8, 2), (8, 8), (2, 8)], layer=202, datatype=1))
    gds_path = tmp_path / "fixture.gds"
    library.write_gds(gds_path)

    stack_path = tmp_path / "fixture.stack.json"
    stack_path.write_text(
        json.dumps(
            {
                "metadata": {
                    "top_cell_name": "TOP",
                    "ignored_layout_layers": [
                        {"layer": 202, "datatype": 1, "name": "D0_TOP_SIM_BOUNDARY"}
                    ],
                },
                "solution_regions": {
                    "AIR": {
                        "role": "solution_region",
                        "material_id": "AIR",
                        "geometry": {"z_min_um": 0.0, "z_max_um": 10.0},
                    }
                },
                "layers": [
                    {
                        "layer": 1,
                        "datatype": 0,
                        "semantic_id": "HOST_METAL",
                        "role": "metal",
                        "material_id": "aluminum",
                        "geometry": {
                            "geometry_source": "gds_polygon",
                            "z_um": 0.0,
                            "thickness_um": 0.2,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    build_input = build_gds_stack_geometry_input(
        gds_file=gds_path,
        stack_file=stack_path,
    )

    assert build_input.port_sheet_regions == ()


def test_port_sheet_without_host_overlap_records_empty_overlap_set(tmp_path) -> None:
    library = gdstk.Library()
    cell = library.new_cell("TOP")
    cell.add(gdstk.Polygon([(0, 0), (1, 0), (1, 1), (0, 1)], layer=1, datatype=0))
    cell.add(gdstk.Polygon([(2, 2), (3, 2), (3, 3), (2, 3)], layer=202, datatype=1))
    gds_path = tmp_path / "fixture.gds"
    library.write_gds(gds_path)

    stack_path = tmp_path / "fixture.stack.json"
    stack_path.write_text(
        json.dumps(
            {
                "metadata": {
                    "top_cell_name": "TOP",
                    "port_sheet_source_layers": [
                        {
                            "layer": 202,
                            "datatype": 1,
                            "name": "P1",
                            "source": "palace_lumped_port_sheet",
                        }
                    ],
                },
                "solution_regions": {
                    "AIR": {
                        "role": "solution_region",
                        "material_id": "AIR",
                        "geometry": {"z_min_um": 0.0, "z_max_um": 10.0},
                    }
                },
                "layers": [
                    {
                        "layer": 1,
                        "datatype": 0,
                        "semantic_id": "HOST_METAL",
                        "role": "metal",
                        "material_id": "aluminum",
                        "route_representations": {
                            "A": "surface_sheet",
                            "B": "cutout_boundary_shell",
                            "C": "material_volume",
                        },
                        "geometry": {
                            "geometry_source": "gds_polygon",
                            "z_um": 0.0,
                            "thickness_um": 0.2,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    build_input = build_gds_stack_geometry_input(
        gds_file=gds_path,
        stack_file=stack_path,
    )

    assert len(build_input.port_sheet_regions) == 1
    assert build_input.port_sheet_regions[0].overlaps == ()
    validate_geometry_input(build_input)


def test_port_sheet_region_rejects_non_local_fragment_operation() -> None:
    build_input = GeometryBuildInput(
        polygons=(
            LayoutPolygonSpec(
                polygon_id="HOST__P0000",
                layer="1/0",
                exterior=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
            ),
        ),
        entities=(
            SemanticEntitySpec(
                semantic_id="AIR",
                role="solution_region",
                material_id="AIR",
                priority=0,
                geometry_kind="domain",
            ),
            SemanticEntitySpec(
                semantic_id="HOST",
                role="metal",
                material_id="aluminum",
                priority=0,
                geometry_kind="layout_extrusion",
                polygon_ids=("HOST__P0000",),
                route_representations={
                    "A": "surface_sheet",
                    "B": "cutout_boundary_shell",
                    "C": "material_volume",
                },
            ),
        ),
        port_sheet_regions=(
            PortSheetRegionRecord(
                port_sheet_id="PORT",
                source_layer="202/1",
                source_polygon_id="PORT__P0000",
                exterior=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
                overlaps=(
                    PortSheetOverlapRecord(
                        overlap_id="BAD",
                        port_sheet_id="PORT",
                        port_polygon_id="PORT__P0000",
                        host_semantic_id="HOST",
                        host_polygon_id="HOST__P0000",
                        overlap_loop=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
                        operation="global_fragment",
                    ),
                ),
                metadata={"source": "palace_lumped_port_sheet"},
            ),
        ),
    )

    try:
        validate_geometry_input(build_input)
    except ValueError as exc:
        assert "unsupported port-sheet operation" in str(exc)
        return
    raise AssertionError("validate_geometry_input did not fail")


def test_pipeline_stops_before_backend_for_port_sheet_plan(
    tmp_path,
    monkeypatch,
) -> None:
    region = PortSheetRegionRecord(
        port_sheet_id="PORT",
        source_layer="202/1",
        source_polygon_id="PORT__P0000",
        exterior=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
        overlaps=(
            PortSheetOverlapRecord(
                overlap_id="OVERLAP",
                port_sheet_id="PORT",
                port_polygon_id="PORT__P0000",
                host_semantic_id="HOST",
                host_polygon_id="HOST__P0000",
                overlap_loop=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
            ),
        ),
        metadata={"source": "palace_lumped_port_sheet"},
    )
    build_input = GeometryBuildInput(
        polygons=(),
        entities=(
            SemanticEntitySpec(
                semantic_id="AIR",
                role="solution_region",
                material_id="AIR",
                priority=0,
                geometry_kind="domain",
            ),
        ),
    )

    def fake_plan(*_args, **_kwargs):
        return ConstructionPlanRecord(route="A", port_sheet_regions=(region,))

    monkeypatch.setattr(
        "semantic_geometry_builder.pipeline.build_route_construction_plan",
        fake_plan,
    )

    try:
        SemanticGeometryBuilder().build(
            build_input,
            route="A",
            run_folder=tmp_path / "run",
        )
    except NotImplementedError as exc:
        assert "port_sheet_regions contract" in str(exc)
    else:
        raise AssertionError("pipeline did not stop before backend")

    assert (
        tmp_path
        / "run"
        / "metadata"
        / "semantic_geometry"
        / "02_build_route_construction_plan.json"
    ).is_file()
