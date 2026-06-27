from semantic_geometry_builder.models import (
    CurvePlanRecord,
    CurveRefRecord,
    PointPlanRecord,
    SurfaceLoopRecord,
    SurfacePlanRecord,
)
from semantic_geometry_builder.validation import (
    validate_curve_plan_coverage,
    validate_no_surface_overlap,
)


def _point(point_id: str, x: float, y: float) -> PointPlanRecord:
    return PointPlanRecord(point_id=point_id, coordinate=(x, y, 0.0))


def _curve(curve_id: str, start: str, end: str) -> CurvePlanRecord:
    return CurvePlanRecord(
        curve_id=curve_id,
        curve_kind="line_segment",
        start_point_id=start,
        end_point_id=end,
    )


def _surface(loop_id: str = "L0") -> SurfacePlanRecord:
    return SurfacePlanRecord(
        surface_id="S0",
        owner_semantic_id="owner",
        surface_role="test",
        geometry_ref={"outer_loop": ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))},
        outer_loop_ref=loop_id,
    )


def _assert_validation_error(expected: str, **kwargs: object) -> None:
    try:
        validate_curve_plan_coverage(**kwargs)  # type: ignore[arg-type]
    except ValueError as exc:
        assert expected in str(exc)
        return
    raise AssertionError("validate_curve_plan_coverage did not fail")


def test_valid_square_loop_passes() -> None:
    points = (
        _point("P0", 0.0, 0.0),
        _point("P1", 1.0, 0.0),
        _point("P2", 1.0, 1.0),
        _point("P3", 0.0, 1.0),
    )
    curves = (
        _curve("C0", "P0", "P1"),
        _curve("C1", "P1", "P2"),
        _curve("C2", "P2", "P3"),
        _curve("C3", "P0", "P3"),
    )
    loops = (
        SurfaceLoopRecord(
            loop_id="L0",
            curve_refs=(
                CurveRefRecord("C0", 1, "boundary"),
                CurveRefRecord("C1", 1, "boundary"),
                CurveRefRecord("C2", 1, "boundary"),
                CurveRefRecord("C3", -1, "boundary"),
            ),
            role="outer",
            surface_id="S0",
        ),
    )
    validate_curve_plan_coverage(
        points=points,
        curves=curves,
        surface_loops=loops,
        surfaces=(_surface(),),
    )


def test_repeated_curve_ref_fails() -> None:
    points = (
        _point("P0", 0.0, 0.0),
        _point("P1", 1.0, 0.0),
        _point("P2", 1.0, 1.0),
        _point("P3", 0.0, 1.0),
        _point("P4", 2.0, 0.0),
        _point("P5", 2.0, 1.0),
    )
    curves = (
        _curve("C0", "P0", "P1"),
        _curve("C1", "P1", "P2"),
        _curve("C2", "P2", "P3"),
        _curve("C3", "P0", "P3"),
        _curve("C4", "P1", "P4"),
        _curve("C5", "P4", "P5"),
        _curve("C6", "P0", "P5"),
    )
    loops = (
        SurfaceLoopRecord(
            loop_id="L0",
            curve_refs=(
                CurveRefRecord("C0", 1, "boundary"),
                CurveRefRecord("C1", 1, "boundary"),
                CurveRefRecord("C2", 1, "boundary"),
                CurveRefRecord("C3", -1, "boundary"),
                CurveRefRecord("C0", 1, "boundary"),
                CurveRefRecord("C4", 1, "boundary"),
                CurveRefRecord("C5", 1, "boundary"),
                CurveRefRecord("C6", -1, "boundary"),
            ),
            role="outer",
            surface_id="S0",
        ),
    )
    _assert_validation_error(
        "repeats curve refs",
        points=points,
        curves=curves,
        surface_loops=loops,
        surfaces=(_surface(),),
    )


def test_self_touching_point_fails() -> None:
    points = (
        _point("P0", 0.0, 0.0),
        _point("P1", 1.0, 0.0),
        _point("P2", 0.0, 1.0),
        _point("P3", -1.0, 0.0),
        _point("P4", 0.0, -1.0),
    )
    curves = (
        _curve("C0", "P0", "P1"),
        _curve("C1", "P1", "P2"),
        _curve("C2", "P0", "P2"),
        _curve("C3", "P0", "P3"),
        _curve("C4", "P3", "P4"),
        _curve("C5", "P0", "P4"),
    )
    loops = (
        SurfaceLoopRecord(
            loop_id="L0",
            curve_refs=(
                CurveRefRecord("C0", 1, "boundary"),
                CurveRefRecord("C1", 1, "boundary"),
                CurveRefRecord("C2", -1, "boundary"),
                CurveRefRecord("C3", 1, "boundary"),
                CurveRefRecord("C4", 1, "boundary"),
                CurveRefRecord("C5", -1, "boundary"),
            ),
            role="outer",
            surface_id="S0",
        ),
    )
    _assert_validation_error(
        "not a simple surface loop",
        points=points,
        curves=curves,
        surface_loops=loops,
        surfaces=(_surface(),),
    )


def test_no_surface_overlap_skips_exact_known_hole() -> None:
    shell = SurfacePlanRecord(
        surface_id="shell",
        owner_semantic_id="owner",
        surface_role="test",
        geometry_ref={
            "plane": {"axis": "z", "value_um": 0.0},
            "outer_loop": ((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)),
            "hole_loops": (((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)),),
        },
    )
    hole_surface = SurfacePlanRecord(
        surface_id="hole_surface",
        owner_semantic_id="owner",
        surface_role="test",
        geometry_ref={
            "plane": {"axis": "z", "value_um": 0.0},
            "outer_loop": ((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)),
        },
    )
    validate_no_surface_overlap(surfaces=(shell, hole_surface))


def test_no_surface_overlap_rejects_real_overlap() -> None:
    first = SurfacePlanRecord(
        surface_id="first",
        owner_semantic_id="owner",
        surface_role="test",
        geometry_ref={
            "plane": {"axis": "z", "value_um": 0.0},
            "outer_loop": ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)),
        },
    )
    second = SurfacePlanRecord(
        surface_id="second",
        owner_semantic_id="owner",
        surface_role="test",
        geometry_ref={
            "plane": {"axis": "z", "value_um": 0.0},
            "outer_loop": ((1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)),
        },
    )
    try:
        validate_no_surface_overlap(surfaces=(first, second))
    except ValueError as exc:
        assert "overlaps" in str(exc)
        return
    raise AssertionError("validate_no_surface_overlap did not fail")


if __name__ == "__main__":
    test_valid_square_loop_passes()
    test_repeated_curve_ref_fails()
    test_self_touching_point_fails()
    test_no_surface_overlap_skips_exact_known_hole()
    test_no_surface_overlap_rejects_real_overlap()
