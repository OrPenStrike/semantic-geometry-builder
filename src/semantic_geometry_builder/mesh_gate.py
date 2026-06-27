"""Diagnostic Mesh Gate for SGB XAO handoff investigations.

This module is intentionally outside `SemanticGeometryBuilder.build()`: SGB owns
semantic CAD topology, while downstream consumers own production meshing and
solver config. The Mesh Gate exists to reproduce gsim-like tetra meshing from
an already-written SGB XAO, check that sidecar physical groups still match the
live Gmsh model, and reject invalid tetra topology before anyone treats the
mesh as Palace-ready.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter, defaultdict
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from semantic_geometry_builder.models import PathInput

DEFAULT_MESH_GATE_PROFILES = ("gsim_default", "hxt_10")
_SCHEMA = "sgb.mesh_gate.v1"
_REFINED_MESH_SIZE_UM = 5.0
_MAX_MESH_SIZE_UM = 300.0
_VOLUME6_TOL_UM3 = 0.0


def run_mesh_gate(
    xao_path: PathInput,
    *,
    physical_groups_path: PathInput | None = None,
    output_dir: PathInput | None = None,
    profiles: Sequence[str] = DEFAULT_MESH_GATE_PROFILES,
    write_meshes: bool = False,
    terminal: bool = False,
) -> dict[str, Any]:
    """Run diagnostic mesh profiles against one SGB XAO.

    The gate reads the SGB `04_export_physical_groups.json` sidecar, opens the
    XAO in Gmsh, applies each mesh profile in a fresh Gmsh session, generates a
    3D mesh, and checks tetra zero-volume plus face-incidence topology. It does
    not write Palace `config.json`; that file should remain downstream and
    should be written only after this topology gate passes.
    """
    xao = Path(xao_path)
    groups_path = (
        Path(physical_groups_path)
        if physical_groups_path is not None
        else _default_physical_groups_path(xao)
    )
    reports = []
    for profile in profiles:
        report = _run_mesh_gate_profile(
            xao,
            groups_path,
            profile=profile,
            output_dir=Path(output_dir) if output_dir is not None else None,
            write_mesh=write_meshes,
            terminal=terminal,
        )
        reports.append(report)
    return {
        "schema": _SCHEMA,
        "mesh_gate": "mesh_safe_conformal",
        "status": (
            "fail"
            if any(report["status"] != "pass" for report in reports)
            else "pass"
        ),
        "xao_path": str(xao),
        "physical_groups_path": str(groups_path),
        "profiles": [report["profile"] for report in reports],
        "reports": reports,
    }


def _run_mesh_gate_profile(
    xao_path: Path,
    physical_groups_path: Path,
    *,
    profile: str,
    output_dir: Path | None,
    write_mesh: bool,
    terminal: bool,
) -> dict[str, Any]:
    if profile not in DEFAULT_MESH_GATE_PROFILES:
        raise ValueError(f"unknown Mesh Gate profile: {profile!r}")

    started = time.perf_counter()
    failures: list[dict[str, Any]] = []
    records = _load_physical_group_records(physical_groups_path, failures)
    report: dict[str, Any] = _base_report(
        xao_path=xao_path,
        physical_groups_path=physical_groups_path,
        profile=profile,
    )
    gmsh = None
    was_initialized = False
    try:
        import gmsh as gmsh_module

        gmsh = gmsh_module
        was_initialized = bool(gmsh.isInitialized())
        if not was_initialized:
            gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 1 if terminal else 0)
        gmsh.clear()
        gmsh.open(str(xao_path))
        profile_options = _apply_mesh_profile(gmsh, profile)
        group_check = _check_physical_groups(gmsh, records, failures)
        refinement = _setup_gsim_like_refinement(
            gmsh,
            group_check["surface_entity_tags"],
            refined_mesh_size_um=_REFINED_MESH_SIZE_UM,
            max_mesh_size_um=_MAX_MESH_SIZE_UM,
        )
        gmsh.model.mesh.generate(3)
        mesh_stats = _collect_mesh_stats(gmsh)
        topology = _audit_gmsh_tetra_topology(gmsh)
        _append_topology_failures(topology, failures)
        mesh_path = None
        if write_mesh:
            mesh_path = _write_mesh(gmsh, output_dir, xao_path, profile)
        report.update(
            {
                "profile_options": profile_options,
                "physical_group_check": group_check,
                "refinement": refinement,
                "mesh_stats": mesh_stats,
                "tetra_topology": topology,
                "mesh_path": str(mesh_path) if mesh_path is not None else None,
            }
        )
    except Exception as exc:
        failures.append(_failure("mesh_gate_execution_failed", (), str(exc)))
    finally:
        if gmsh is not None:
            gmsh.clear()
            if not was_initialized:
                gmsh.finalize()

    report["seconds"] = round(time.perf_counter() - started, 6)
    report["failures"] = failures
    report["status"] = "fail" if failures else "pass"
    report["config_policy"] = {
        "status": "pass" if not failures else "blocked",
        "config_written_by_gate": False,
        "message": "Mesh Gate never writes Palace config.json.",
    }
    report["palace_amr_check"] = {
        "status": "not_run",
        "message": "Palace load/AMR is downstream of this XAO mesh topology gate.",
    }
    _write_report(output_dir, report)
    return report


def _base_report(
    *,
    xao_path: Path,
    physical_groups_path: Path,
    profile: str,
) -> dict[str, Any]:
    return {
        "schema": _SCHEMA,
        "mesh_gate": "mesh_safe_conformal",
        "profile": profile,
        "status": "fail",
        "xao_path": str(xao_path),
        "physical_groups_path": str(physical_groups_path),
        "tolerances": {"volume6_um3": _VOLUME6_TOL_UM3},
    }


def _apply_mesh_profile(gmsh: Any, profile: str) -> dict[str, Any]:
    options = {
        "Mesh.MeshSizeMin": _REFINED_MESH_SIZE_UM,
        "Mesh.MeshSizeMax": _MAX_MESH_SIZE_UM,
        "Mesh.MeshSizeExtendFromBoundary": 0,
        "Mesh.MeshSizeFromPoints": 0,
        "Mesh.MeshSizeFromCurvature": 0,
        "Mesh.Algorithm": 5,
    }
    if profile == "hxt_10":
        options.update(
            {
                "General.NumThreads": 10,
                "Mesh.MaxNumThreads1D": 10,
                "Mesh.MaxNumThreads2D": 10,
                "Mesh.MaxNumThreads3D": 10,
                "Mesh.Algorithm3D": 10,
            }
        )
    for name, value in options.items():
        gmsh.option.setNumber(name, value)
    return options


def _load_physical_group_records(
    path: Path,
    failures: list[dict[str, Any]],
) -> tuple[Mapping[str, Any], ...]:
    if not path.is_file():
        failures.append(
            _failure(
                "missing_physical_group_sidecar",
                (str(path),),
                "Mesh Gate needs 04_export_physical_groups.json.",
            )
        )
        return ()
    payload = json.loads(path.read_text())
    if not isinstance(payload, list):
        failures.append(
            _failure(
                "invalid_physical_group_sidecar",
                (str(path),),
                "04_export_physical_groups.json must contain a JSON list.",
            )
        )
        return ()
    return tuple(record for record in payload if isinstance(record, Mapping))


def _check_physical_groups(
    gmsh: Any,
    records: Sequence[Mapping[str, Any]],
    failures: list[dict[str, Any]],
) -> dict[str, Any]:
    live_entities = {
        dim: {int(tag) for _, tag in gmsh.model.getEntities(dim)}
        for dim in (2, 3)
    }
    live_groups: dict[tuple[int, str], tuple[int, tuple[int, ...]]] = {}
    for dim, group_tag in gmsh.model.getPhysicalGroups():
        name = gmsh.model.getPhysicalName(dim, group_tag)
        if not name:
            continue
        entity_tags = tuple(
            int(tag) for tag in gmsh.model.getEntitiesForPhysicalGroup(dim, group_tag)
        )
        live_groups[(int(dim), name)] = (int(group_tag), entity_tags)

    checked_records = []
    surface_entity_tags: set[int] = set()
    for record in records:
        if record.get("solver_use", "solver_active") != "solver_active":
            continue
        name = str(record.get("physical_name", ""))
        dim = int(record.get("dimension", 0) or 0)
        sidecar_tags = tuple(int(tag) for tag in record.get("entity_tags", ()))
        live_group = live_groups.get((dim, name))
        record_failures = []
        if live_group is None:
            record_failures.append("missing_physical_group")
            failures.append(
                _failure(
                    "missing_physical_group",
                    (name,),
                    f"XAO has no live dim={dim} physical group named {name!r}.",
                )
            )
            actual_tags: tuple[int, ...] = ()
        else:
            _, actual_tags = live_group
            if sidecar_tags and len(actual_tags) != len(sidecar_tags):
                record_failures.append("physical_group_entity_count_mismatch")
                failures.append(
                    _failure(
                        "physical_group_entity_count_mismatch",
                        (name,),
                        "Live XAO group has a different entity count than sidecar.",
                    )
                )
        stale_tags = tuple(
            tag for tag in actual_tags if tag not in live_entities.get(dim, set())
        )
        if stale_tags:
            record_failures.append("stale_live_entity_tag")
            failures.append(
                _failure(
                    "stale_live_entity_tag",
                    (name,),
                    f"Physical group references non-live dim={dim} tags {stale_tags}.",
                )
            )
        if dim == 2:
            surface_entity_tags.update(actual_tags)
        checked_records.append(
            {
                "physical_name": name,
                "dimension": dim,
                "sidecar_entity_tags": sidecar_tags,
                "live_entity_tags": actual_tags,
                "status": "fail" if record_failures else "pass",
                "failures": record_failures,
            }
        )

    if records and not any(record.get("dimension") == 3 for record in records):
        failures.append(
            _failure(
                "missing_volume_group_intent",
                (),
                "Sidecar has no volume physical-group records.",
            )
        )
    return {
        "status": "fail"
        if any(record["status"] != "pass" for record in checked_records)
        else "pass",
        "records": checked_records,
        "surface_entity_tags": sorted(surface_entity_tags),
        "counts": {
            "sidecar_records": len(records),
            "checked_solver_active_records": len(checked_records),
            "live_physical_groups": len(live_groups),
            "refinement_surface_entities": len(surface_entity_tags),
        },
    }


def _setup_gsim_like_refinement(
    gmsh: Any,
    surface_tags: Sequence[int],
    *,
    refined_mesh_size_um: float,
    max_mesh_size_um: float,
) -> dict[str, Any]:
    boundary_curve_tags: set[int] = set()
    for surface_tag in surface_tags:
        try:
            boundary = gmsh.model.getBoundary(
                [(2, int(surface_tag))],
                combined=False,
                oriented=False,
                recursive=False,
            )
        except Exception:
            continue
        boundary_curve_tags.update(int(tag) for dim, tag in boundary if dim == 1)
    if not boundary_curve_tags:
        return {
            "status": "skipped",
            "surface_entity_count": len(surface_tags),
            "boundary_curve_count": 0,
            "field_ids": (),
        }

    distance_field = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(
        distance_field,
        "CurvesList",
        sorted(boundary_curve_tags),
    )
    gmsh.model.mesh.field.setNumber(distance_field, "Sampling", 200)

    threshold_field = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(threshold_field, "InField", distance_field)
    gmsh.model.mesh.field.setNumber(threshold_field, "SizeMin", refined_mesh_size_um)
    gmsh.model.mesh.field.setNumber(threshold_field, "SizeMax", max_mesh_size_um)
    gmsh.model.mesh.field.setNumber(threshold_field, "DistMin", 0)
    gmsh.model.mesh.field.setNumber(threshold_field, "DistMax", max_mesh_size_um)

    min_field = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(min_field, "FieldsList", [threshold_field])
    gmsh.model.mesh.field.setAsBackgroundMesh(min_field)

    return {
        "status": "applied",
        "surface_entity_count": len(surface_tags),
        "boundary_curve_count": len(boundary_curve_tags),
        "field_ids": (distance_field, threshold_field, min_field),
        "refined_mesh_size_um": refined_mesh_size_um,
        "max_mesh_size_um": max_mesh_size_um,
    }


def _collect_mesh_stats(gmsh: Any) -> dict[str, Any]:
    stats: dict[str, Any] = {}
    try:
        node_tags, _, _ = gmsh.model.mesh.getNodes()
        stats["nodes"] = len(node_tags)
    except Exception:
        pass
    try:
        element_types, element_tags, _ = gmsh.model.mesh.getElements()
        stats["elements"] = sum(len(tags) for tags in element_tags)
        stats["tetrahedra"] = 0
        for element_type, tags in zip(element_types, element_tags, strict=False):
            name, dim, *_ = gmsh.model.mesh.getElementProperties(int(element_type))
            if int(dim) == 3 and "tetra" in str(name).lower():
                stats["tetrahedra"] += len(tags)
    except Exception:
        pass
    tet_tags = _tetra_element_tags(gmsh)
    if tet_tags:
        stats["quality"] = _quality_summary(gmsh, tet_tags, "gamma")
        stats["sicn"] = _quality_summary(gmsh, tet_tags, "minSICN")
        stats["sige"] = _quality_summary(gmsh, tet_tags, "minSIGE")
    return {key: value for key, value in stats.items() if value not in ({}, None)}


def _tetra_element_tags(gmsh: Any) -> list[int]:
    result: list[int] = []
    element_types, element_tags, _ = gmsh.model.mesh.getElements(3)
    for element_type, tags in zip(element_types, element_tags, strict=False):
        name, dim, *_ = gmsh.model.mesh.getElementProperties(int(element_type))
        if int(dim) == 3 and "tetra" in str(name).lower():
            result.extend(int(tag) for tag in tags)
    return result


def _quality_summary(
    gmsh: Any,
    element_tags: Sequence[int],
    metric: str,
) -> dict[str, Any]:
    try:
        values = [
            float(value)
            for value in gmsh.model.mesh.getElementQualities(element_tags, metric)
        ]
    except Exception:
        return {}
    if not values:
        return {}
    return {
        "min": min(values),
        "mean": sum(values) / len(values),
        "invalid_below_zero": sum(1 for value in values if value < 0.0),
    }


def _audit_gmsh_tetra_topology(gmsh: Any) -> dict[str, Any]:
    node_points = _mesh_node_points(gmsh)
    tet_tags, tet_corners = _mesh_tetra_corners(gmsh)
    return _audit_tetra_topology(
        node_points=node_points,
        tet_tags=tet_tags,
        tet_corners=tet_corners,
        volume6_tolerance=_VOLUME6_TOL_UM3,
    )


def _mesh_node_points(gmsh: Any) -> dict[int, tuple[float, float, float]]:
    node_tags, coords, _ = gmsh.model.mesh.getNodes()
    return {
        int(tag): (
            float(coords[index * 3]),
            float(coords[index * 3 + 1]),
            float(coords[index * 3 + 2]),
        )
        for index, tag in enumerate(node_tags)
    }


def _mesh_tetra_corners(
    gmsh: Any,
) -> tuple[list[int], list[tuple[int, int, int, int]]]:
    element_types, element_tags, element_nodes = gmsh.model.mesh.getElements(3)
    tet_tags: list[int] = []
    tet_corners: list[tuple[int, int, int, int]] = []
    for element_type, tags, nodes in zip(
        element_types,
        element_tags,
        element_nodes,
        strict=False,
    ):
        name, dim, _order, node_count, _coords, primary_count = (
            gmsh.model.mesh.getElementProperties(int(element_type))
        )
        if int(dim) != 3 or "tetra" not in str(name).lower():
            continue
        if int(primary_count) < 4:
            continue
        stride = int(node_count)
        for index, tag in enumerate(tags):
            start = index * stride
            corners = tuple(int(node) for node in nodes[start : start + 4])
            if len(corners) != 4:
                continue
            tet_tags.append(int(tag))
            tet_corners.append(corners)
    return tet_tags, tet_corners


def _audit_tetra_topology(
    *,
    node_points: Mapping[int, tuple[float, float, float]],
    tet_tags: Sequence[int],
    tet_corners: Sequence[tuple[int, int, int, int]],
    volume6_tolerance: float,
) -> dict[str, Any]:
    faces: defaultdict[tuple[int, int, int], int] = defaultdict(int)
    zero_volume_tets: list[int] = []
    face_indices = ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3))
    for tet_tag, corners in zip(tet_tags, tet_corners, strict=False):
        try:
            a, b, c, d = (node_points[node] for node in corners)
        except KeyError:
            zero_volume_tets.append(int(tet_tag))
            continue
        volume6 = _tetra_volume6(a, b, c, d)
        if abs(volume6) <= volume6_tolerance:
            zero_volume_tets.append(int(tet_tag))
        for indices in face_indices:
            faces[tuple(sorted(corners[index] for index in indices))] += 1
    bad_face_counts = Counter(count for count in faces.values() if count > 2)
    return {
        "status": "pass"
        if tet_corners and not zero_volume_tets and not bad_face_counts
        else "fail",
        "tetrahedra": len(tet_corners),
        "zero_volume_tetra_count": len(zero_volume_tets),
        "zero_volume_tetra_tags_sample": zero_volume_tets[:20],
        "face_incidence_gt2_count": sum(bad_face_counts.values()),
        "face_incidence_gt2_histogram": dict(sorted(bad_face_counts.items())),
    }


def _tetra_volume6(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
    d: tuple[float, float, float],
) -> float:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    ad = (d[0] - a[0], d[1] - a[1], d[2] - a[2])
    return (
        ab[0] * (ac[1] * ad[2] - ac[2] * ad[1])
        - ab[1] * (ac[0] * ad[2] - ac[2] * ad[0])
        + ab[2] * (ac[0] * ad[1] - ac[1] * ad[0])
    )


def _append_topology_failures(
    topology: Mapping[str, Any],
    failures: list[dict[str, Any]],
) -> None:
    if topology.get("tetrahedra", 0) <= 0:
        failures.append(
            _failure("missing_tetrahedra", (), "Gmsh produced no tetrahedra.")
        )
    if topology.get("zero_volume_tetra_count", 0):
        failures.append(
            _failure(
                "zero_volume_tetrahedra",
                (),
                f"{topology['zero_volume_tetra_count']} tetrahedra have zero volume.",
            )
        )
    if topology.get("face_incidence_gt2_count", 0):
        failures.append(
            _failure(
                "tetra_face_incidence_gt2",
                (),
                (
                    f"{topology['face_incidence_gt2_count']} triangular faces "
                    "are shared by more than two tetrahedra."
                ),
            )
        )


def _write_mesh(
    gmsh: Any,
    output_dir: Path | None,
    xao_path: Path,
    profile: str,
) -> Path:
    out_dir = output_dir or xao_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    mesh_path = out_dir / f"{xao_path.stem}.{profile}.msh"
    gmsh.option.setNumber("Mesh.Binary", 0)
    gmsh.option.setNumber("Mesh.SaveAll", 0)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.write(str(mesh_path))
    return mesh_path


def _write_report(output_dir: Path | None, report: Mapping[str, Any]) -> None:
    if output_dir is None:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / f"mesh_gate_{report['profile']}.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


def _default_physical_groups_path(xao_path: Path) -> Path:
    if xao_path.parent.name == "geometry":
        return (
            xao_path.parent.parent
            / "metadata"
            / "semantic_geometry"
            / "04_export_physical_groups.json"
        )
    return xao_path.parent / "04_export_physical_groups.json"


def _default_output_dir(xao_path: Path, run_folder: Path | None) -> Path:
    if run_folder is not None:
        return run_folder / "metadata" / "semantic_geometry"
    if xao_path.parent.name == "geometry":
        return xao_path.parent.parent / "metadata" / "semantic_geometry"
    return xao_path.parent


def _xao_from_run_folder(run_folder: Path, route: str | None) -> Path:
    geometry_dir = run_folder / "geometry"
    if route:
        return geometry_dir / f"semantic_geometry_route_{route.lower()}.xao"
    matches = sorted(geometry_dir.glob("semantic_geometry_route_*.xao"))
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one route XAO in {geometry_dir}, found {len(matches)}."
        )
    return matches[0]


def _failure(code: str, record_ids: Sequence[str], message: str) -> dict[str, Any]:
    return {
        "code": code,
        "record_ids": list(record_ids),
        "message": message,
    }


def _self_check() -> None:
    nodes = {
        1: (0.0, 0.0, 0.0),
        2: (1.0, 0.0, 0.0),
        3: (0.0, 1.0, 0.0),
        4: (0.0, 0.0, 1.0),
        5: (0.0, 0.0, -1.0),
        6: (1.0, 1.0, 0.0),
    }
    ok = _audit_tetra_topology(
        node_points=nodes,
        tet_tags=(10,),
        tet_corners=((1, 2, 3, 4),),
        volume6_tolerance=0.0,
    )
    assert ok["status"] == "pass", ok
    flat = _audit_tetra_topology(
        node_points=nodes,
        tet_tags=(11,),
        tet_corners=((1, 2, 3, 6),),
        volume6_tolerance=0.0,
    )
    assert flat["zero_volume_tetra_count"] == 1, flat
    non_manifold = _audit_tetra_topology(
        node_points=nodes,
        tet_tags=(12, 13, 14),
        tet_corners=((1, 2, 3, 4), (1, 2, 3, 5), (1, 2, 3, 6)),
        volume6_tolerance=0.0,
    )
    assert non_manifold["face_incidence_gt2_count"] == 1, non_manifold


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xao_path", nargs="?", help="SGB route XAO path.")
    parser.add_argument("--run-folder", type=Path, help="SGB run folder.")
    parser.add_argument("--route", choices=("A", "B", "C"), help="Route XAO to load.")
    parser.add_argument(
        "--physical-groups",
        type=Path,
        help="04_export_physical_groups.json path.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory for mesh_gate_<profile>.json reports.",
    )
    parser.add_argument(
        "--profile",
        action="append",
        choices=DEFAULT_MESH_GATE_PROFILES,
        help="Profile to run. Repeat to select multiple profiles.",
    )
    parser.add_argument("--write-mesh", action="store_true", help="Write .msh files.")
    parser.add_argument("--terminal", action="store_true", help="Show Gmsh terminal.")
    parser.add_argument("--self-check", action="store_true", help="Run logic checks.")
    args = parser.parse_args(argv)

    if args.self_check:
        _self_check()
        return 0
    if args.xao_path is None and args.run_folder is None:
        parser.error("provide xao_path or --run-folder")
    run_folder = Path(args.run_folder) if args.run_folder is not None else None
    xao_path = Path(args.xao_path) if args.xao_path else _xao_from_run_folder(
        run_folder or Path(),
        args.route,
    )
    output_dir = args.output_dir or _default_output_dir(xao_path, run_folder)
    summary = run_mesh_gate(
        xao_path,
        physical_groups_path=args.physical_groups,
        output_dir=output_dir,
        profiles=tuple(args.profile or DEFAULT_MESH_GATE_PROFILES),
        write_meshes=args.write_mesh,
        terminal=args.terminal,
    )
    print(json.dumps(_summary_for_stdout(summary, output_dir), indent=2))
    return 0 if summary["status"] == "pass" else 1


def _summary_for_stdout(summary: Mapping[str, Any], output_dir: Path) -> dict[str, Any]:
    return {
        "status": summary["status"],
        "xao_path": summary["xao_path"],
        "physical_groups_path": summary["physical_groups_path"],
        "output_dir": str(output_dir),
        "profiles": [
            {
                "profile": report["profile"],
                "status": report["status"],
                "failures": report["failures"],
                "report_path": str(output_dir / f"mesh_gate_{report['profile']}.json"),
            }
            for report in summary["reports"]
        ],
    }


if __name__ == "__main__":
    raise SystemExit(main())
