from semantic_geometry_builder import (
    GeometryBuildInput,
    LayoutPolygonSpec,
    SemanticEntitySpec,
)


def test_public_api_records_are_importable() -> None:
    polygon = LayoutPolygonSpec(
        polygon_id="poly",
        layer="D0_TOP_M1",
        exterior=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
    )
    entity = SemanticEntitySpec(
        semantic_id="metal",
        role="conductor",
        material_id="Al",
        priority=10,
        geometry_kind="layout_extrusion",
        polygon_ids=(polygon.polygon_id,),
    )

    build_input = GeometryBuildInput(polygons=(polygon,), entities=(entity,))

    assert build_input.entities[0].semantic_id == "metal"
