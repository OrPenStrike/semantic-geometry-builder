"""Frontend adapter contracts that build GeometryBuildInput records."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType
from typing import TYPE_CHECKING, Any, TypeAlias

from semantic_geometry_builder.models import (
    GeometryBuildInput,
    LayoutPolygonSpec,
    PathInput,
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


def build_gds_stack_geometry_input(
    *,
    gds_file: PathInput,
    stack_file: PathInput,
    top_cell_name: str | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> GeometryBuildInput:
    """Level 0 adapter: build GeometryBuildInput from GDS plus stack JSON.

    This is the canonical frontend contract for v1. Tool-specific adapters
    such as GDSFactory, gsim, KQCircuits, or direct KLayout support should
    lower into this same semantic stack shape instead of inventing their own
    geometry semantics.

    `gds_file` is the layout source path. The adapter must load this file,
    select `top_cell_name` when provided, apply an explicit hierarchy/flattening
    policy, and convert layer/datatype shapes to `LayoutPolygonSpec` records
    without leaking raw KLayout objects.

    `top_cell_name` optionally selects the GDS cell to adapt. If it is omitted
    and a later KLayout-backed extraction sees multiple plausible top cells, it
    should fail fast rather than guessing silently.

    `stack_file` is a path to a project stackup JSON file. It is the reviewed
    semantic contract for layer/datatype, material, z-range, route
    representation, and interface-recognition metadata. Unsupported suffixes
    fail clearly.

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
    - `metadata`: optional mapping copied into `GeometryBuildInput.metadata`.

    `metadata` is copied to `GeometryBuildInput.metadata` for adapter version,
    source GDS/stack file paths, selected top cell, unit convention, KLayout
    database unit, flattening policy, and stack-file dialect/provenance.

    The implementation must return a fully frontend-normalized
    `GeometryBuildInput`: `polygons` from KLayout geometry, `entities` from the
    resolved stack-file material semantics, and `solution_regions` from
    solver-domain definitions such as AIR, substrate, dielectric, or enclosure
    boxes. It must not emit solver config or assume Ansys physical names are the
    final semantic ids.
    """
    import gdstk

    gds_path = Path(gds_file)
    if not gds_path.is_file():
        raise FileNotFoundError(gds_path)

    stack_mapping, stack_path = _load_stack_mapping(stack_file)
    raw_layers = stack_mapping.get("layers")
    if not _is_record_sequence(raw_layers):
        raise TypeError("stack_file must define sequence 'layers'")

    solution_regions = stack_mapping.get("solution_regions")
    if not isinstance(solution_regions, Mapping):
        raise TypeError("stack_file must define mapping 'solution_regions'")

    stack_metadata = stack_mapping.get("metadata", {})
    if not isinstance(stack_metadata, Mapping):
        raise TypeError("stack_file 'metadata' must be a mapping when provided")
    if metadata is not None and not isinstance(metadata, Mapping):
        raise TypeError("metadata must be a mapping when provided")

    library = gdstk.read_gds(str(gds_path))
    cell = _select_gds_cell(library, top_cell_name)
    cell_bounds = _cell_bounds_um(cell)
    polygons_by_layer = _polygons_by_layer(cell)

    polygons: list[LayoutPolygonSpec] = []
    entities: list[SemanticEntitySpec] = [
        _solution_region_entity_from_record(
            semantic_id,
            record,
            cell_bounds_um=cell_bounds,
        )
        for semantic_id, record in solution_regions.items()
    ]
    domain_bounds_by_semantic_id = {
        entity.semantic_id: entity.geometry["domain_bounds_um"]
        for entity in entities
        if isinstance(entity.geometry.get("domain_bounds_um"), Mapping)
    }

    for record in raw_layers:
        entity, entity_polygons = _entity_and_polygons_from_layer_record(
            record,
            polygons_by_layer=polygons_by_layer,
            cell_bounds_um=cell_bounds,
            domain_bounds_by_semantic_id=domain_bounds_by_semantic_id,
        )
        polygons.extend(entity_polygons)
        entities.append(entity)

    combined_metadata = {
        **dict(stack_metadata),
        **dict(metadata or {}),
        "adapter": "gds_stack",
        "gds_file": str(gds_path),
        "stack_file": str(stack_path),
        "selected_cell_name": cell.name,
        "cell_bounds_um": cell_bounds,
    }
    combined_metadata["interface_intents_2d"] = _route_a_sheet_interfaces(
        entities,
        polygons,
    )

    return GeometryBuildInput(
        polygons=tuple(polygons),
        entities=tuple(entities),
        solution_regions=dict(solution_regions),
        metadata=combined_metadata,
    )


def build_gdsfactory_geometry_input(
    *,
    component: Component,
    layer_stack: LayerStack,
    materials: Mapping[str, Any] | None = None,
    metadata: Mapping[str, Any] | None = None,
    top_cell_name: str | None = None,
    work_dir: PathInput | None = None,
    padding_um: float = 100.0,
) -> GeometryBuildInput:
    """Build GeometryBuildInput from GDSFactory layout and technology objects.

    This adapter intentionally lowers GDSFactory/gsim objects into the reviewed
    Level-0 contract: a GDS file plus stack JSON, then delegates to
    `build_gds_stack_geometry_input`. The adapter does not invent a second
    semantic language.

    `layer_stack` must expose `layers` and `dielectrics` like gsim's
    `LayerStack`. Only conductor/via layout layers become semantic metal
    entities in this first slice; solution regions come from `dielectrics`.
    Missing GDS layer, z-range, material, air/vacuum region, or route
    representation data fails fast.

    `work_dir` makes the generated GDS and stack JSON reviewable. Without it,
    temporary files are used only long enough to call the Level-0 adapter.
    """
    if materials is not None and not isinstance(materials, Mapping):
        raise TypeError("materials must be a mapping when provided")

    if work_dir is None:
        with TemporaryDirectory() as tmp:
            return _build_gdsfactory_geometry_input_from_dir(
                component=component,
                layer_stack=layer_stack,
                materials=materials,
                metadata=metadata,
                top_cell_name=top_cell_name,
                work_dir=Path(tmp),
                padding_um=padding_um,
            )
    return _build_gdsfactory_geometry_input_from_dir(
        component=component,
        layer_stack=layer_stack,
        materials=materials,
        metadata=metadata,
        top_cell_name=top_cell_name,
        work_dir=Path(work_dir),
        padding_um=padding_um,
    )


def build_kqcircuits_geometry_input(
    *,
    cell: kdb.Cell,
    layer_config: KQCircuitsLayerConfig,
    layer_properties: Any | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> GeometryBuildInput:
    """Build GeometryBuildInput from KQCircuits/KLayout layout and layer config.

    This is a secondary adapter. It should translate KQCircuits/KLayout layer
    config, cell geometry, and face policy into the same Level 0 semantic stack
    contract used by `build_gds_stack_geometry_input`. KQCircuits-specific
    layer names or face groupings should become reviewed stack semantics before
    the geometry builder sees them.

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
    resolved Level 0 stack/face semantics, and selected route metadata only
    when it is explicit in the project rules. Unknown layer names, missing face
    mappings, or ambiguous material ownership should fail fast.
    """
    del cell, layer_config, layer_properties, metadata
    raise NotImplementedError("build_kqcircuits_geometry_input")


def _build_gdsfactory_geometry_input_from_dir(
    *,
    component: Any,
    layer_stack: Any,
    materials: Mapping[str, Any] | None,
    metadata: Mapping[str, Any] | None,
    top_cell_name: str | None,
    work_dir: Path,
    padding_um: float,
) -> GeometryBuildInput:
    work_dir.mkdir(parents=True, exist_ok=True)
    component_name = str(getattr(component, "name", "component") or "component")
    safe_name = component_name.replace("/", "_").replace(":", "_")
    gds_path = work_dir / f"{safe_name}.gds"
    stack_path = work_dir / f"{safe_name}.stack.json"

    write_gds = getattr(component, "write_gds", None)
    if write_gds is None:
        raise TypeError("component must provide write_gds(path)")
    try:
        write_gds(gds_path)
    except TypeError:
        write_gds(str(gds_path))

    stack_mapping = _semantic_stack_mapping_from_layer_stack(
        layer_stack,
        materials=materials,
        source_gds=gds_path,
        padding_um=padding_um,
    )
    stack_path.write_text(
        json.dumps(stack_mapping, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    return build_gds_stack_geometry_input(
        gds_file=gds_path,
        stack_file=stack_path,
        top_cell_name=top_cell_name or component_name,
        metadata={
            **dict(metadata or {}),
            "adapter": "gdsfactory",
            "component_name": component_name,
            "generated_gds_file": str(gds_path),
            "generated_stack_file": str(stack_path),
        },
    )


def _semantic_stack_mapping_from_layer_stack(
    layer_stack: Any,
    *,
    materials: Mapping[str, Any] | None,
    source_gds: Path,
    padding_um: float,
) -> dict[str, Any]:
    layers = getattr(layer_stack, "layers", None)
    if not isinstance(layers, Mapping):
        raise TypeError("layer_stack must expose a mapping 'layers'")

    solution_regions = _solution_regions_from_layer_stack(
        layer_stack,
        padding_um=padding_um,
    )
    host_void_semantic_id = _first_air_like_solution_id(solution_regions)
    layer_records = [
        _semantic_layer_record(name, layer, host_void_semantic_id)
        for name, layer in layers.items()
        if _layer_type(layer) in {"conductor", "via"}
    ]
    if not layer_records:
        raise ValueError("layer_stack has no conductor/via layers to adapt")

    return {
        "metadata": {
            "schema": "semantic_geometry_stack_v1",
            "units": "um",
            "source": str(source_gds),
            "adapter": "gdsfactory",
            "material_names": sorted(str(key) for key in (materials or {}).keys()),
        },
        "solution_regions": solution_regions,
        "layers": layer_records,
    }


def _solution_regions_from_layer_stack(
    layer_stack: Any,
    *,
    padding_um: float,
) -> dict[str, Any]:
    raw_dielectrics = getattr(layer_stack, "dielectrics", None)
    if not _is_record_sequence(raw_dielectrics):
        raise TypeError("layer_stack must expose sequence 'dielectrics'")

    regions: dict[str, Any] = {}
    for index, raw in enumerate(raw_dielectrics):
        if not isinstance(raw, Mapping):
            raise TypeError("layer_stack.dielectrics items must be mappings")
        material_id = str(raw.get("material") or raw.get("material_id") or "")
        if not material_id:
            raise ValueError("dielectric records must define material")
        semantic_id = str(raw.get("name") or raw.get("domain") or material_id or index)
        z_min = raw.get("zmin", raw.get("z_min_um"))
        z_max = raw.get("zmax", raw.get("z_max_um"))
        if z_min is None or z_max is None:
            raise ValueError(f"solution region {semantic_id!r} needs zmin/zmax")
        regions[semantic_id] = {
            "role": "solution_region",
            "material_id": material_id,
            "geometry_kind": "domain",
            "geometry": {
                "domain": semantic_id,
                "padding_um": padding_um,
                "z_min_um": float(z_min),
                "z_max_um": float(z_max),
            },
        }
    return regions


def _semantic_layer_record(
    name: Any,
    layer: Any,
    host_void_semantic_id: str,
) -> dict[str, Any]:
    gds_layer = _gds_layer_tuple(layer)
    layer_type = _layer_type(layer)
    z_min = getattr(layer, "zmin", None)
    thickness = getattr(layer, "thickness", None)
    material_id = str(getattr(layer, "material", "") or "")
    if z_min is None or thickness is None:
        raise ValueError(f"layer {name!r} needs zmin/thickness")
    if not material_id:
        raise ValueError(f"layer {name!r} needs material")

    semantic_id = str(name)
    is_via = layer_type == "via"
    return {
        "layer": gds_layer[0],
        "datatype": gds_layer[1],
        "semantic_id": semantic_id,
        "role": "metal",
        "material_id": material_id,
        "priority": int(getattr(layer, "mesh_order", 0) or 0),
        "part_role": "bump_body" if is_via else "face_metal",
        "net_id": semantic_id,
        "geometry_kind": "layout_extrusion",
        "host_void_semantic_id": host_void_semantic_id,
        "geometry": {
            "z_um": float(z_min),
            "thickness_um": float(thickness),
            "geometry_source": "gds_polygon",
        },
        "route_representations": (
            {
                "A": "cutout_boundary_shell",
                "B": "cutout_boundary_shell",
                "C": "material_volume",
            }
            if is_via
            else {
                "A": "surface_sheet",
                "B": "cutout_boundary_shell",
                "C": "material_volume",
            }
        ),
        "metadata": {
            "source_layer_name": semantic_id,
            "source_layer_type": layer_type,
        },
    }


def _gds_layer_tuple(layer: Any) -> tuple[int, int]:
    value = getattr(layer, "gds_layer", None)
    if value is None:
        value = getattr(layer, "layer", None)
    if (
        isinstance(value, Sequence)
        and not isinstance(value, str | bytes)
        and len(value) == 2
    ):
        return int(value[0]), int(value[1])
    raise ValueError("layer must define gds_layer as a 2-item tuple")


def _layer_type(layer: Any) -> str:
    value = getattr(layer, "layer_type", None)
    if value is None:
        info = getattr(layer, "info", None)
        if isinstance(info, Mapping):
            value = info.get("layer_type")
    if value is None:
        raise ValueError("layer must define layer_type")
    return str(value)


def _first_air_like_solution_id(solution_regions: Mapping[str, Any]) -> str:
    for semantic_id, record in solution_regions.items():
        material = str(record.get("material_id", "")).lower()
        name = str(semantic_id).lower()
        if any(token in f"{name} {material}" for token in ("air", "vacuum")):
            return str(semantic_id)
    raise ValueError("layer_stack dielectrics must include an air/vacuum region")


def _load_stack_mapping(
    stack_file: PathInput,
) -> tuple[Mapping[str, Any], Path]:
    stack_path = Path(stack_file)
    if not stack_path.is_file():
        raise FileNotFoundError(stack_path)
    if stack_path.suffix.lower() != ".json":
        raise ValueError(
            "unsupported stack_file suffix "
            f"{stack_path.suffix!r}; only JSON is supported for now"
        )

    data = json.loads(stack_path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise TypeError("JSON stack_file root must be a mapping")
    return data, stack_path


def _is_record_sequence(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(value, str | bytes)


def _select_gds_cell(library: Any, top_cell_name: str | None) -> Any:
    if top_cell_name is not None:
        for cell in library.cells:
            if cell.name == top_cell_name:
                return cell
        raise ValueError(f"GDS top_cell_name not found: {top_cell_name!r}")

    candidates = [
        cell
        for cell in library.cells
        if cell.name != "$$$CONTEXT_INFO$$$"
        and cell.get_polygons(apply_repetitions=True)
    ]
    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        names = ", ".join(sorted(cell.name for cell in candidates))
        raise ValueError(f"top_cell_name is required; candidates: {names}")

    top_cells = [
        cell
        for cell in library.top_level()
        if cell.get_polygons(apply_repetitions=True)
    ]
    if len(top_cells) == 1:
        return top_cells[0]
    names = ", ".join(sorted(cell.name for cell in top_cells))
    raise ValueError(f"top_cell_name is required; candidates: {names}")


def _cell_bounds_um(cell: Any) -> dict[str, float]:
    bbox = cell.bounding_box()
    if bbox is None:
        raise ValueError(f"GDS cell {cell.name!r} has no bounding box")
    (x_min, y_min), (x_max, y_max) = bbox
    return {
        "x_min_um": float(x_min),
        "y_min_um": float(y_min),
        "x_max_um": float(x_max),
        "y_max_um": float(y_max),
    }


def _polygons_by_layer(cell: Any) -> dict[tuple[int, int], tuple[Any, ...]]:
    result: dict[tuple[int, int], list[Any]] = {}
    for polygon in cell.get_polygons(apply_repetitions=True):
        result.setdefault(
            (int(polygon.layer), int(polygon.datatype)),
            [],
        ).append(polygon)
    return {key: tuple(value) for key, value in result.items()}


def _solution_region_entity_from_record(
    semantic_id: Any,
    record: Any,
    *,
    cell_bounds_um: Mapping[str, float],
) -> SemanticEntitySpec:
    if not isinstance(semantic_id, str):
        raise TypeError("solution region ids must be strings")
    if not isinstance(record, Mapping):
        raise TypeError("stack_file 'solution_regions' values must be mappings")

    geometry = record.get("geometry", record)
    if not isinstance(geometry, Mapping):
        raise TypeError("solution region 'geometry' must be a mapping")

    geometry = dict(geometry)
    padding_um = float(geometry.get("padding_um", 0.0))
    geometry.setdefault(
        "domain_bounds_um",
        {
            "x_min_um": cell_bounds_um["x_min_um"] - padding_um,
            "y_min_um": cell_bounds_um["y_min_um"] - padding_um,
            "x_max_um": cell_bounds_um["x_max_um"] + padding_um,
            "y_max_um": cell_bounds_um["y_max_um"] + padding_um,
        },
    )

    return SemanticEntitySpec(
        semantic_id=semantic_id,
        role=record.get("role", "solution_region"),
        material_id=record.get("material_id", semantic_id),
        priority=record.get("priority", 0),
        geometry_kind=record.get("geometry_kind", "domain"),
        geometry=geometry,
        metadata=record.get("metadata", {}),
    )


def _entity_and_polygons_from_layer_record(
    record: Any,
    *,
    polygons_by_layer: Mapping[tuple[int, int], tuple[Any, ...]],
    cell_bounds_um: Mapping[str, float],
    domain_bounds_by_semantic_id: Mapping[str, Mapping[str, Any]],
) -> tuple[SemanticEntitySpec, tuple[LayoutPolygonSpec, ...]]:
    entity = _entity_from_layer_record(record)
    geometry_source = str(entity.geometry.get("geometry_source", "gds_polygon"))
    if geometry_source == "die_face_minus_ground_mask":
        entity_polygons = _derived_ground_polygons(
            entity,
            polygons_by_layer=polygons_by_layer,
            cell_bounds_um=cell_bounds_um,
            domain_bounds_by_semantic_id=domain_bounds_by_semantic_id,
        )
    elif geometry_source == "gds_polygon":
        entity_polygons = _gds_polygons_for_entity(
            entity,
            polygons_by_layer=polygons_by_layer,
        )
    else:
        entity_polygons = ()

    entity = _entity_with_polygon_geometry(entity, entity_polygons)
    return entity, entity_polygons


def _entity_from_layer_record(record: Any) -> SemanticEntitySpec:
    if not isinstance(record, Mapping):
        raise TypeError("stack_file 'layers' items must be mappings")

    layer = record.get("layer")
    datatype = record.get("datatype")
    if layer is None or datatype is None:
        raise ValueError("stack_file layer records must define 'layer' and 'datatype'")

    for required_field in ("semantic_id", "role", "material_id"):
        if required_field not in record:
            raise ValueError(
                f"stack_file layer records must define {required_field!r}"
            )

    raw_geometry = record.get("geometry", {})
    if not isinstance(raw_geometry, Mapping):
        raise TypeError("stack_file layer record 'geometry' must be a mapping")
    geometry = dict(raw_geometry)
    if "z_um" in record or "thickness_um" in record:
        if "z_um" not in record or "thickness_um" not in record:
            raise ValueError(
                "stack_file layer records must define both 'z_um' and "
                "'thickness_um', or use 'geometry'"
            )
        geometry.setdefault("z_um", record["z_um"])
        geometry.setdefault("thickness_um", record["thickness_um"])
    if not geometry:
        raise ValueError(
            "stack_file layer records must define 'geometry' or "
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
        raise TypeError("stack_file layer record 'polygon_ids' must be a sequence")
    if isinstance(labels, str | bytes) or not isinstance(labels, Sequence):
        raise TypeError("stack_file layer record 'labels' must be a sequence")

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


def _gds_polygons_for_entity(
    entity: SemanticEntitySpec,
    *,
    polygons_by_layer: Mapping[tuple[int, int], tuple[Any, ...]],
) -> tuple[LayoutPolygonSpec, ...]:
    layer = int(entity.geometry["gds_layer"])
    datatype = int(entity.geometry["gds_datatype"])
    candidates = polygons_by_layer.get((layer, datatype), ())
    if not candidates:
        return ()

    selector = entity.geometry.get("selector_point_um")
    if selector is not None:
        selected = [
            polygon
            for polygon in candidates
            if _point_in_ring(
                (float(selector[0]), float(selector[1])),
                _ring_from_gdstk_polygon(polygon),
            )
        ]
        if len(selected) != 1:
            raise ValueError(
                f"{entity.semantic_id} selector_point_um matched "
                f"{len(selected)} polygons"
            )
    else:
        selected = list(candidates)

    return tuple(
        LayoutPolygonSpec(
            polygon_id=f"{entity.semantic_id}__P{index:04d}",
            layer=f"{layer}/{datatype}",
            exterior=_ring_from_gdstk_polygon(polygon),
            object_name=entity.semantic_id,
            net_name=entity.net_id,
            metadata={
                "gds_layer": layer,
                "gds_datatype": datatype,
                "source": "gds_polygon",
            },
        )
        for index, polygon in enumerate(selected)
    )


def _derived_ground_polygons(
    entity: SemanticEntitySpec,
    *,
    polygons_by_layer: Mapping[tuple[int, int], tuple[Any, ...]],
    cell_bounds_um: Mapping[str, float],
    domain_bounds_by_semantic_id: Mapping[str, Mapping[str, Any]],
) -> tuple[LayoutPolygonSpec, ...]:
    mask_layer = entity.geometry.get("mask_layer")
    if mask_layer is None:
        mask_key = (
            int(entity.geometry["gds_layer"]),
            int(entity.geometry["gds_datatype"]),
        )
    else:
        mask_key = (int(mask_layer[0]), int(mask_layer[1]))
    holes = tuple(
        _ring_from_gdstk_polygon(polygon)
        for polygon in polygons_by_layer.get(mask_key, ())
    )
    exterior = _rectangle_ring(
        _ground_plane_bounds(entity, domain_bounds_by_semantic_id, cell_bounds_um)
    )
    return (
        LayoutPolygonSpec(
            polygon_id=f"{entity.semantic_id}__P0000",
            layer=f"{mask_key[0]}/{mask_key[1]}",
            exterior=exterior,
            holes=holes,
            object_name=entity.semantic_id,
            net_name=entity.net_id,
            metadata={
                "gds_layer": mask_key[0],
                "gds_datatype": mask_key[1],
                "source": "die_face_minus_ground_mask",
            },
        ),
    )


def _ground_plane_bounds(
    entity: SemanticEntitySpec,
    domain_bounds_by_semantic_id: Mapping[str, Mapping[str, Any]],
    cell_bounds_um: Mapping[str, float],
) -> Mapping[str, Any]:
    plane_bounds_ref = entity.geometry.get("plane_bounds_ref")
    if plane_bounds_ref is None:
        return cell_bounds_um
    if not isinstance(plane_bounds_ref, str):
        raise TypeError(f"{entity.semantic_id} plane_bounds_ref must be a string")
    try:
        return domain_bounds_by_semantic_id[plane_bounds_ref]
    except KeyError as exc:
        raise ValueError(
            f"{entity.semantic_id} plane_bounds_ref {plane_bounds_ref!r} "
            "does not match a solution region"
        ) from exc


def _entity_with_polygon_geometry(
    entity: SemanticEntitySpec,
    polygons: tuple[LayoutPolygonSpec, ...],
) -> SemanticEntitySpec:
    if not polygons:
        return entity
    geometry = dict(entity.geometry)
    if len(polygons) == 1:
        geometry.setdefault("outer_loop", polygons[0].exterior)
        geometry.setdefault("hole_loops", polygons[0].holes)
    polygon_ids = tuple(polygon.polygon_id for polygon in polygons)
    return SemanticEntitySpec(
        semantic_id=entity.semantic_id,
        role=entity.role,
        material_id=entity.material_id,
        priority=entity.priority,
        geometry_kind=entity.geometry_kind,
        part_role=entity.part_role,
        attached_face_metal_semantic_id=entity.attached_face_metal_semantic_id,
        net_id=entity.net_id,
        polygon_ids=polygon_ids,
        labels=entity.labels,
        host_void_semantic_id=entity.host_void_semantic_id,
        requires_construction_body=entity.requires_construction_body,
        route_representations=entity.route_representations,
        geometry=geometry,
        metadata=entity.metadata,
    )


def _route_a_sheet_interfaces(
    entities: Sequence[SemanticEntitySpec],
    polygons: Sequence[LayoutPolygonSpec],
) -> dict[str, tuple[Mapping[str, Any], ...]]:
    polygons_by_id = {polygon.polygon_id: polygon for polygon in polygons}
    interfaces: list[Mapping[str, Any]] = []
    for entity in entities:
        if entity.route_representations.get("A") != "surface_sheet":
            continue
        for index, polygon_id in enumerate(entity.polygon_ids):
            polygon = polygons_by_id[polygon_id]
            z_um = float(entity.geometry.get("z_um", 0.0))
            interfaces.append(
                {
                    "interface_id": f"MA__{entity.semantic_id}__AIR__{index:04d}",
                    "kind": "MA",
                    "owner_semantic_ids": (entity.semantic_id, "AIR"),
                    "interface_kinds": ("MS", "MA"),
                    "recognition_rule": "route_a_surface_sheet_polygon",
                    "source_polygon_ids": (polygon_id,),
                    "valid_routes": ("A",),
                    "plane": {"axis": "z", "value_um": z_um},
                    "outer_loop": polygon.exterior,
                    "hole_loops": polygon.holes,
                }
            )
    return {"interfaces": tuple(interfaces)}


def _ring_from_gdstk_polygon(polygon: Any) -> tuple[tuple[float, float], ...]:
    points = tuple((float(x), float(y)) for x, y in polygon.points)
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("GDS polygon requires at least 3 unique points")
    return points


def _rectangle_ring(bounds: Mapping[str, float]) -> tuple[tuple[float, float], ...]:
    return (
        (float(bounds["x_min_um"]), float(bounds["y_min_um"])),
        (float(bounds["x_max_um"]), float(bounds["y_min_um"])),
        (float(bounds["x_max_um"]), float(bounds["y_max_um"])),
        (float(bounds["x_min_um"]), float(bounds["y_max_um"])),
    )


def _point_in_ring(
    point: tuple[float, float],
    ring: Sequence[tuple[float, float]],
) -> bool:
    x, y = point
    inside = False
    j = len(ring) - 1
    for i, (xi, yi) in enumerate(ring):
        xj, yj = ring[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_intersect:
                inside = not inside
        j = i
    return inside
