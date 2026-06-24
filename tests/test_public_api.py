from dataclasses import fields
from inspect import Parameter, signature
from typing import get_args

from semantic_geometry_builder import (
    RUN_METADATA_DIR,
    SEMANTIC_GEOMETRY_METADATA_DIR,
    CutHostOperationRecord,
    FinalPhysicalGroupRecord,
    GeometryBuildInput,
    InterfacePatchRecord,
    LayoutPolygonSpec,
    RingPatchRecord,
    RouteMaterializationRecord,
    RoutePolicyRecord,
    SemanticEntitySpec,
    SemanticGeometryBuilder,
    build_gdsfactory_geometry_input,
    build_klayout_tech_geometry_input,
    build_kqcircuits_geometry_input,
    validate_geometry_input,
)
from semantic_geometry_builder.adapter import (
    build_gdsfactory_geometry_input as adapter_build_gdsfactory_geometry_input,
)
from semantic_geometry_builder.models import (
    DEFAULT_INTERFACE_SOLVER_USE,
    RingApplicationModeLiteral,
)
from semantic_geometry_builder.models import (
    GeometryBuildInput as ModelGeometryBuildInput,
)
from semantic_geometry_builder.pipeline import (
    validate_geometry_input as pipeline_validate_geometry_input,
)
from semantic_geometry_builder.route_materialization.context import (
    build_route_materialization_context,
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
        route_representations={"A": "surface_sheet"},
    )

    air = SemanticEntitySpec(
        semantic_id="AIR",
        role="solution_region",
        material_id="AIR",
        priority=0,
        geometry_kind="domain",
    )

    build_input = GeometryBuildInput(polygons=(polygon,), entities=(air, entity))

    assert build_input.entities[0].semantic_id == "AIR"
    assert build_input.entities[1].semantic_id == "metal"


def test_layers_reexport_same_contract_objects() -> None:
    assert GeometryBuildInput is ModelGeometryBuildInput
    assert validate_geometry_input is pipeline_validate_geometry_input
    assert build_gdsfactory_geometry_input is adapter_build_gdsfactory_geometry_input
    assert callable(build_kqcircuits_geometry_input)
    assert callable(build_klayout_tech_geometry_input)


def test_builder_build_requires_run_folder_contract() -> None:
    parameters = signature(SemanticGeometryBuilder.build).parameters

    assert parameters["run_folder"].kind is Parameter.KEYWORD_ONLY
    assert RUN_METADATA_DIR == "metadata"
    assert SEMANTIC_GEOMETRY_METADATA_DIR == "semantic_geometry"


def test_public_records_use_solver_neutral_contract_names() -> None:
    contract_classes = (
        SemanticEntitySpec,
        InterfacePatchRecord,
        RouteMaterializationRecord,
        FinalPhysicalGroupRecord,
    )

    for contract_class in contract_classes:
        names = {field.name for field in fields(contract_class)}
        assert not any("palace" in name.lower() for name in names)

    assert "solver_use" in {field.name for field in fields(InterfacePatchRecord)}
    assert "solver_use" in {field.name for field in fields(FinalPhysicalGroupRecord)}


def test_route_b_shell_and_inset_contract_shape() -> None:
    policy = RoutePolicyRecord(
        route="B",
        conductor_union_kind="cutout_boundary_shell",
    )
    interface = InterfacePatchRecord(
        interface_id="AIRBRIDGE_DECK_TOP_MA",
        kind="MA",
        owner_semantic_ids=("airbridge_deck", "AIR"),
        adjacent_atomic_volume_ids=("deck_ref", "air_ref"),
    )
    ring = RingPatchRecord(
        ring_id="AIRBRIDGE_DECK_TOP_MA__BAND_0_50NM",
        parent_interface_id=interface.interface_id,
        label="BAND_0_50NM",
        band_min_um=0.0,
        band_max_um=0.05,
        valid_routes=("B",),
    )
    route_record = RouteMaterializationRecord(
        route="B",
        conductor_union_kind=policy.conductor_union_kind,
        pec_boundary_interface_ids=(interface.interface_id,),
        surviving_ring_ids=(ring.ring_id,),
        boundary_shell_groups={"PEC_AIRBRIDGE": (interface.interface_id,)},
    )

    assert route_record.route == "B"
    assert route_record.conductor_union_kind == "cutout_boundary_shell"
    assert route_record.pec_boundary_interface_ids == (interface.interface_id,)
    assert route_record.boundary_shell_groups["PEC_AIRBRIDGE"] == (
        "AIRBRIDGE_DECK_TOP_MA",
    )
    assert get_args(RingApplicationModeLiteral) == ("replace_parent_with_children",)


def test_construction_body_is_separate_from_representation() -> None:
    bump = SemanticEntitySpec(
        semantic_id="indium_bump",
        role="conductor",
        material_id="indium",
        priority=20,
        geometry_kind="bump",
        part_role="bump_body",
        requires_construction_body=True,
        route_representations={
            "A": "cutout_boundary_shell",
            "B": "cutout_boundary_shell",
            "C": "material_volume",
        },
    )

    assert bump.requires_construction_body
    assert bump.route_representations["A"] == "cutout_boundary_shell"


def test_cut_host_operation_splits_interface_kind_from_surface_role() -> None:
    operation = CutHostOperationRecord(
        operation_id="cut_indium_bump_from_air",
        construction_body_id="indium_bump_body",
        host_solution_volume_id="AIR",
        expected_exposed_interface_kinds=("MA",),
        expected_exposed_surface_roles=("sidewall", "top_cap"),
    )

    assert operation.expected_exposed_interface_kinds == ("MA",)
    assert operation.expected_exposed_surface_roles == ("sidewall", "top_cap")


def test_route_materialization_context_has_all_interface_kind_keys() -> None:
    context = build_route_materialization_context(
        atomic_volumes=(),
        interfaces=(),
        rings=(),
    )

    assert set(context.interfaces_by_kind) == set(DEFAULT_INTERFACE_SOLVER_USE)
    assert context.interfaces_by_kind["MA"] == ()


def test_interface_solver_use_defaults_by_kind() -> None:
    metal_air = InterfacePatchRecord(
        interface_id="AIRBRIDGE_DECK_TOP_MA",
        kind="MA",
        owner_semantic_ids=("airbridge_deck", "AIR"),
        adjacent_atomic_volume_ids=("deck_ref", "air_ref"),
    )
    metal_metal = InterfacePatchRecord(
        interface_id="AIRBRIDGE_POST_TOP_TO_DECK_MM",
        kind="MM",
        owner_semantic_ids=("airbridge_post", "airbridge_deck"),
        adjacent_atomic_volume_ids=("post_ref", "deck_ref"),
    )
    explicitly_postprocessing = InterfacePatchRecord(
        interface_id="AIRBRIDGE_POST_FOOT_MM",
        kind="MM",
        owner_semantic_ids=("airbridge_post", "face_metal"),
        adjacent_atomic_volume_ids=("post_ref", "face_ref"),
        solver_use="postprocessing_only",
    )

    assert metal_air.solver_use == "solver_active"
    assert metal_metal.solver_use == "audit_only"
    assert explicitly_postprocessing.solver_use == "postprocessing_only"


def test_total_physical_group_is_logical_aggregate_shape() -> None:
    total = FinalPhysicalGroupRecord(
        physical_name="TOTAL",
        dimension=3,
        route="C",
        role="logical_total",
        source_record_id="logical_total",
        logical_only=True,
        child_physical_names=("AIR", "substrate", "metal"),
    )

    assert total.logical_only
    assert total.entity_tags == ()
    assert total.child_physical_names == ("AIR", "substrate", "metal")


def test_total_physical_group_rejects_backend_tags() -> None:
    try:
        FinalPhysicalGroupRecord(
            physical_name="TOTAL_MA",
            dimension=3,
            route="C",
            role="logical_total",
            source_record_id="logical_total",
            logical_only=True,
            entity_tags=(1,),
        )
    except ValueError as exc:
        assert "backend entity tags" in str(exc)
    else:
        raise AssertionError("logical-only group must not carry backend entity tags")


def test_logical_total_role_requires_logical_only() -> None:
    try:
        FinalPhysicalGroupRecord(
            physical_name="IF__AIR_METAL_MA__TOTAL",
            dimension=2,
            route="A",
            role="logical_total",
            source_record_id="IF__AIR_METAL_MA",
        )
    except ValueError as exc:
        assert "logical_only" in str(exc)
    else:
        raise AssertionError("logical_total role must be logical_only")
