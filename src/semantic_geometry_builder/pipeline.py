"""Public route-aware semantic geometry pipeline facade.

The heavy semantic work lives in `planning.py` and `validation.py`. This module
keeps the public import path stable and owns only run orchestration: validate,
plan, write sidecars, then hand the plan to the future bottom-up OCC backend.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from semantic_geometry_builder.backends.gmsh_occ_backend import (
    write_occ_geometry_from_plan,
)
from semantic_geometry_builder.export import export_physical_group_records
from semantic_geometry_builder.models import (
    RUN_METADATA_DIR,
    SEMANTIC_GEOMETRY_METADATA_DIR,
    FinalPhysicalGroupRecord,
    GeometryBuildInput,
    PathInput,
    RouteLiteral,
)
from semantic_geometry_builder.planning import (
    build_route_construction_plan,
    plan_cut_host_operations,
    plan_route_construction_bodies,
    plan_route_surfaces,
    plan_route_tags,
    plan_route_volumes,
    plan_surface_partitions,
    recognize_route_interfaces,
)
from semantic_geometry_builder.validation import (
    validate_geometry_input,
    validate_route_operation_coverage,
    validate_route_volume_surface_refs,
    validate_selected_route,
    validate_surface_partition_coverage,
    validate_surface_sheet_interface_coverage,
    validate_tag_plan_coverage,
)

__all__ = [
    "SemanticGeometryBuilder",
    "build_route_construction_plan",
    "export_physical_group_records",
    "plan_cut_host_operations",
    "plan_route_construction_bodies",
    "plan_route_surfaces",
    "plan_route_tags",
    "plan_route_volumes",
    "plan_surface_partitions",
    "recognize_route_interfaces",
    "validate_geometry_input",
    "validate_route_operation_coverage",
    "validate_route_volume_surface_refs",
    "validate_selected_route",
    "validate_surface_partition_coverage",
    "validate_surface_sheet_interface_coverage",
    "validate_tag_plan_coverage",
]


class SemanticGeometryBuilder:
    """Facade for one route-aware semantic geometry construction run.

    The builder must not construct arbitrary volumes and then ask the backend
    to fragment them to discover topology. Route selection is an early input:
    Route A plans sheets/shells, Route B plans cutout shells, and Route C plans
    retained material volumes with explicit shared surfaces where required.

    `build()` is the reviewable top-level pipeline:

    1. validate the adapter-owned `GeometryBuildInput`;
    2. build a route-specific `ConstructionPlanRecord`;
    3. call the bottom-up Gmsh/OCC backend with that plan;
    4. export `FinalPhysicalGroupRecord`s from backend dim-tags.

    One call writes one route XAO file. Mesh generation is downstream.
    """

    def build(
        self,
        build_input: GeometryBuildInput,
        *,
        route: RouteLiteral,
        run_folder: PathInput,
    ) -> tuple[FinalPhysicalGroupRecord, ...]:
        """Validate input, plan geometry, build OCC, then export groups."""
        run_path = Path(run_folder)
        metadata_dir = run_path / RUN_METADATA_DIR / SEMANTIC_GEOMETRY_METADATA_DIR
        geometry_dir = run_path / "geometry"
        geometry_dir.mkdir(parents=True, exist_ok=True)

        validated_input = validate_geometry_input(build_input)
        validate_selected_route(validated_input, route)
        _write_stage_sidecar(
            metadata_dir,
            "01_validate_geometry_input",
            validated_input,
        )

        plan = build_route_construction_plan(validated_input, route=route)
        _write_stage_sidecar(metadata_dir, "02_build_route_construction_plan", plan)

        built_plan = write_occ_geometry_from_plan(
            plan,
            xao_path=geometry_dir / f"semantic_geometry_route_{route.lower()}.xao",
        )
        _write_stage_sidecar(metadata_dir, "03_build_occ_geometry", built_plan)

        physical_groups = export_physical_group_records(built_plan)
        _write_stage_sidecar(
            metadata_dir,
            "04_export_physical_groups",
            physical_groups,
        )
        return physical_groups


def _write_stage_sidecar(path: Path, stage_name: str, payload: Any) -> None:
    path.mkdir(parents=True, exist_ok=True)
    path.joinpath(f"{stage_name}.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True, default=asdict),
        encoding="utf-8",
    )
