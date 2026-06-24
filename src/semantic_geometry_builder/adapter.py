"""Frontend adapter contracts that build GeometryBuildInput records."""

from __future__ import annotations

from collections.abc import Mapping
from types import ModuleType
from typing import TYPE_CHECKING, Any, TypeAlias

from semantic_geometry_builder.models import GeometryBuildInput, PathInput

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
    tech_file: PathInput | Mapping[str, Any],
    top_cell_name: str | None = None,
    layer_properties: Any | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> GeometryBuildInput:
    """Build GeometryBuildInput from a GDS file and a stackup tech file.

    `gds_file` is the layout source path. The adapter is responsible for loading
    it through KLayout, selecting `top_cell_name` when provided, applying an
    explicit hierarchy/flattening policy, and reading shapes from KLayout
    layer/datatype pairs. Returned polygons/entities must not retain raw KLayout
    object references.

    `top_cell_name` optionally selects the GDS cell to adapt. If it is omitted
    and the file has multiple plausible top cells, the implementation should
    fail fast rather than guessing silently.

    `tech_file` is expected to be either a path to a project stackup/technology
    file or a caller-parsed mapping with the same content. This covers
    Ansys/HFSS/Q3D-style tech files without making Ansys a package dependency.
    The adapter uses it to map KLayout layer/datatype pairs to stable semantic
    layer ids, material ids, conductor/dielectric roles, z ranges, thicknesses,
    solution-domain construction metadata, and route representations when the
    tech file states them explicitly. These tech-file dialects are
    project-specific, so ambiguous layer names, missing material ownership,
    unsupported stackup syntax, or implicit route policy should fail fast.

    `layer_properties` optionally carries KLayout `.lyp` display/grouping data.
    It may add audit labels or grouping provenance, but it must not override
    material ownership from the stackup tech file unless a project rule is
    explicit.

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
    del gds_file, tech_file, top_cell_name, layer_properties, metadata
    raise NotImplementedError("build_klayout_tech_geometry_input")
