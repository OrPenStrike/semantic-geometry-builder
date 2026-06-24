"""Frontend adapter contracts that build GeometryBuildInput records."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any, TypeAlias

from semantic_geometry_builder.models import (
    GeometryBuildInput,
    PathInput,
    RoutePolicyRecord,
    SemanticEntitySpec,
)

if TYPE_CHECKING:
    import klayout.db as kdb
    from gdsfactory import Component
    from gdsfactory.technology import LayerStack

KQCircuitsLayerConfig: TypeAlias = (
    ModuleType
    | Mapping[str, "kdb.LayerInfo"]
    | Mapping[str, Mapping[str, "kdb.LayerInfo"]]
)


def build_gdsfactory_geometry_input(
    *,
    component: Component,
    layer_stack: LayerStack,
    materials: Mapping[str, Any] | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> GeometryBuildInput:
    """Build GeometryBuildInput from GDSFactory layout and technology objects.

    `component` is expected to be a `gdsfactory.Component`-like object. The
    adapter uses it only as the layout source: polygons are read per GDS layer
    or layer name and converted to `LayoutPolygonSpec` records. Component ports,
    labels, cell names, or instance provenance may be copied into polygon/entity
    metadata when available, but GDSFactory objects must not leak into the
    returned IR.

    `layer_stack` is expected to be a `gdsfactory.technology.LayerStack`-like
    object whose `layers` map contains `LayerLevel`-like values. The adapter
    uses each layer level's layer expression, derived layer, z-position,
    thickness, material, mesh order, and sidewall/audit fields to assign
    semantic layer ids, material ids, vertical geometry metadata, and ownership
    priority for `SemanticEntitySpec` records.

    `materials` optionally maps frontend material names to canonical
    solver-neutral material metadata. It should refine `material_id` and entity
    metadata only; solver-specific config belongs downstream.

    `metadata` is copied to `GeometryBuildInput.metadata` for adapter version,
    source component name, unit convention, or other audit provenance.

    The implementation should return a fully frontend-normalized
    `GeometryBuildInput`: `polygons` from the component, `entities` from the
    matched layer-stack/material semantics, and route policies only when the
    frontend data makes them unambiguous. Ambiguous layer/material ownership
    should fail fast instead of producing heuristic semantic ids.
    """
    del component, layer_stack, materials, metadata
    raise NotImplementedError("build_gdsfactory_geometry_input")


def build_kqcircuits_geometry_input(
    *,
    cell: kdb.Cell,
    layer_config: KQCircuitsLayerConfig,
    layer_properties: Any | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> GeometryBuildInput:
    """Build GeometryBuildInput from KQCircuits/KLayout layout and layer config.

    `cell` is expected to be a KLayout `pya.Cell`-like object produced by
    KQCircuits. The adapter uses it as the geometry source: shapes are read from
    KLayout layers and converted to `LayoutPolygonSpec` records. The caller or
    adapter must make hierarchy, flattening, and instance provenance explicit in
    metadata; raw KLayout/KQCircuits objects must not leak into the returned IR.

    `layer_config` is expected to be the KQCircuits layer configuration object,
    module, or dict. KQCircuits' default config defines layer names, KLayout
    layer/datatype pairs, and face groupings such as `default_layers` and
    `default_faces`. The adapter uses this to map KLayout layer indices back to
    semantic layer names, chip faces, conductor roles, and stable
    `SemanticEntitySpec` ids.

    `layer_properties` optionally carries KLayout `.lyp` display/grouping data.
    It can add audit labels or visibility/group metadata, but it is not the
    source of physical material ownership unless the adapter has an explicit
    project rule that says so.

    `metadata` is copied to `GeometryBuildInput.metadata` for KQCircuits
    version, layer-config path, face policy, unit convention, or flattening
    policy.

    The implementation should return a fully frontend-normalized
    `GeometryBuildInput`: `polygons` from the KLayout cell, `entities` from the
    resolved KQCircuits layer/face semantics, and route policies only when they
    are explicit in the project rules. Unknown layer names, missing face
    mappings, or ambiguous material ownership should fail fast.
    """
    del cell, layer_config, layer_properties, metadata
    raise NotImplementedError("build_kqcircuits_geometry_input")


def build_klayout_tech_geometry_input(
    *,
    gds_file: PathInput,
    tech_file: PathInput,
    top_cell_name: str | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> GeometryBuildInput:
    """Build GeometryBuildInput from a GDS file and a stackup tech file.

    `gds_file` is the layout source path. This first implementation slice only
    validates that the file exists and records its path as provenance. It does
    not import KLayout or extract shapes yet, so returned `polygons` is empty.
    A later KLayout-backed slice should load this file, select `top_cell_name`
    when provided, apply an explicit hierarchy/flattening policy, and convert
    layer/datatype shapes to `LayoutPolygonSpec` records without leaking raw
    KLayout objects.

    `top_cell_name` optionally selects the GDS cell to adapt. If it is omitted
    and a later KLayout-backed extraction sees multiple plausible top cells, it
    should fail fast rather than guessing silently.

    `tech_file` is a path to a project stackup/technology JSON file. This
    covers Ansys/HFSS/Q3D-style tech-file workflows without making Ansys or
    KLayout a package dependency. Unsupported suffixes fail clearly.

    Minimal JSON schema for this first slice:

    - `layers`: sequence of layer records. Each record maps one GDS
      `layer`/`datatype` pair to a semantic entity using `semantic_id`, `role`,
      `material_id`, optional `priority`, optional `geometry_kind`, optional
      `part_role`, optional `net_id`, optional `polygon_ids`, optional `labels`,
      optional `attached_face_metal_semantic_id`, optional
      `route_representations`, optional `host_void_semantic_id`, and either
      `z_um` plus `thickness_um` or a `geometry` mapping.
    - `solution_regions`: mapping from solution-region semantic id, such as
      `AIR` or `substrate`, to metadata. Each region may define `material_id`,
      `priority`, `geometry_kind`, and `geometry`; missing values default to
      the semantic id, `0`, `domain`, and the metadata mapping itself.
    - `route_policies`: optional sequence of `RoutePolicyRecord` field mappings.
    - `metadata`: optional mapping copied into `GeometryBuildInput.metadata`.

    `metadata` is copied to `GeometryBuildInput.metadata` for adapter version,
    source GDS/tech file paths, selected top cell, unit convention, KLayout
    database unit, flattening policy, and tech-file dialect/provenance.

    The implementation should return a fully frontend-normalized
    `GeometryBuildInput`: `polygons` from KLayout geometry, `entities` from the
    resolved tech-file stackup/material semantics, and `solution_regions` from
    solver-domain definitions such as AIR, substrate, dielectric, or enclosure
    boxes. It must not emit solver config or assume Ansys physical names are the
    final semantic ids.
    """
    gds_path = Path(gds_file)
    if not gds_path.is_file():
        raise FileNotFoundError(gds_path)

    tech_mapping, tech_path = _load_tech_mapping(tech_file)
    raw_layers = tech_mapping.get("layers")
    if not _is_record_sequence(raw_layers):
        raise TypeError("tech_file must define sequence 'layers'")

    solution_regions = tech_mapping.get("solution_regions")
    if not isinstance(solution_regions, Mapping):
        raise TypeError("tech_file must define mapping 'solution_regions'")

    raw_route_policies = tech_mapping.get("route_policies", ())
    if raw_route_policies is None:
        raw_route_policies = ()
    if not _is_record_sequence(raw_route_policies):
        raise TypeError("tech_file 'route_policies' must be a sequence")

    tech_metadata = tech_mapping.get("metadata", {})
    if not isinstance(tech_metadata, Mapping):
        raise TypeError("tech_file 'metadata' must be a mapping when provided")

    build_metadata: dict[str, Any] = {
        "adapter": "klayout_tech",
        "source_gds_file": str(gds_path),
        "tech_file_source": str(tech_path),
        "polygon_extraction": "not_implemented",
    }
    if top_cell_name is not None:
        build_metadata["top_cell_name"] = top_cell_name
    build_metadata.update(tech_metadata)
    build_metadata.update(metadata or {})

    return GeometryBuildInput(
        polygons=(),
        entities=(
            *(
                _solution_region_entity_from_record(semantic_id, record)
                for semantic_id, record in solution_regions.items()
            ),
            *(_entity_from_layer_record(record) for record in raw_layers),
        ),
        route_policies=tuple(
            _route_policy_from_record(record) for record in raw_route_policies
        ),
        solution_regions=solution_regions,
        metadata=build_metadata,
    )


def _load_tech_mapping(
    tech_file: PathInput,
) -> tuple[Mapping[str, Any], Path]:
    tech_path = Path(tech_file)
    if not tech_path.is_file():
        raise FileNotFoundError(tech_path)
    if tech_path.suffix.lower() != ".json":
        raise ValueError(
            "unsupported tech_file suffix "
            f"{tech_path.suffix!r}; only JSON is supported for now"
        )

    data = json.loads(tech_path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise TypeError("JSON tech_file root must be a mapping")
    return data, tech_path


def _is_record_sequence(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(value, str | bytes)


def _solution_region_entity_from_record(
    semantic_id: Any,
    record: Any,
) -> SemanticEntitySpec:
    if not isinstance(semantic_id, str):
        raise TypeError("solution region ids must be strings")
    if not isinstance(record, Mapping):
        raise TypeError("tech_file 'solution_regions' values must be mappings")

    geometry = record.get("geometry", record)
    if not isinstance(geometry, Mapping):
        raise TypeError("solution region 'geometry' must be a mapping")

    return SemanticEntitySpec(
        semantic_id=semantic_id,
        role=record.get("role", "solution_region"),
        material_id=record.get("material_id", semantic_id),
        priority=record.get("priority", 0),
        geometry_kind=record.get("geometry_kind", "domain"),
        geometry=geometry,
        metadata=record.get("metadata", {}),
    )


def _entity_from_layer_record(record: Any) -> SemanticEntitySpec:
    if not isinstance(record, Mapping):
        raise TypeError("tech_file 'layers' items must be mappings")

    layer = record.get("layer")
    datatype = record.get("datatype")
    if layer is None or datatype is None:
        raise ValueError("tech_file layer records must define 'layer' and 'datatype'")

    for required_field in ("semantic_id", "role", "material_id"):
        if required_field not in record:
            raise ValueError(
                f"tech_file layer records must define {required_field!r}"
            )

    raw_geometry = record.get("geometry", {})
    if not isinstance(raw_geometry, Mapping):
        raise TypeError("tech_file layer record 'geometry' must be a mapping")
    geometry = dict(raw_geometry)
    if "z_um" in record or "thickness_um" in record:
        if "z_um" not in record or "thickness_um" not in record:
            raise ValueError(
                "tech_file layer records must define both 'z_um' and "
                "'thickness_um', or use 'geometry'"
            )
        geometry.setdefault("z_um", record["z_um"])
        geometry.setdefault("thickness_um", record["thickness_um"])
    if not geometry:
        raise ValueError(
            "tech_file layer records must define 'geometry' or "
            "'z_um' plus 'thickness_um'"
        )
    geometry.setdefault("gds_layer", layer)
    geometry.setdefault("gds_datatype", datatype)

    polygon_ids = record.get("polygon_ids", ())
    labels = record.get("labels", ())
    if (
        isinstance(polygon_ids, str | bytes)
        or not isinstance(polygon_ids, Sequence)
    ):
        raise TypeError("tech_file layer record 'polygon_ids' must be a sequence")
    if isinstance(labels, str | bytes) or not isinstance(labels, Sequence):
        raise TypeError("tech_file layer record 'labels' must be a sequence")

    return SemanticEntitySpec(
        semantic_id=record["semantic_id"],
        role=record["role"],
        material_id=record["material_id"],
        priority=record.get("priority", 0),
        geometry_kind=record.get("geometry_kind", "layout_extrusion"),
        part_role=record.get("part_role"),
        attached_face_metal_semantic_id=record.get("attached_face_metal_semantic_id"),
        net_id=record.get("net_id"),
        polygon_ids=tuple(polygon_ids),
        labels=tuple(labels),
        host_void_semantic_id=record.get("host_void_semantic_id"),
        requires_construction_body=record.get("requires_construction_body", False),
        route_representations=record.get("route_representations", {}),
        geometry=geometry,
        metadata=record.get("metadata", {}),
    )


def _route_policy_from_record(record: Any) -> RoutePolicyRecord:
    if not isinstance(record, Mapping):
        raise TypeError("tech_file 'route_policies' items must be mappings")
    return RoutePolicyRecord(**dict(record))
