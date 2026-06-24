"""Semantic geometry IR and fail-fast route pipeline.

Responsibility:
Owns: layout-tool-agnostic geometry records, semantic identity, interface
registry records, route materialization records, and final physical-group
plans.
Does not own: PDK loading, GDSFactory/KQCircuits objects, Gmsh implementation,
solver config, reports, notebooks, or run-folder policy.
Inputs: normalized polygons, semantic entities, solution regions, and route
policies from adapters.
Outputs: final topology and physical-group records for mesh/solver consumers.
Pipeline position: between layout-tool adapters and mesh backend adapters.
Source of Truth: semantic entity identity and reference adjacency, not temporary
Gmsh tags or layout-shape heuristics.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

RouteLiteral = Literal["A", "B", "C"]
InterfaceKindLiteral = Literal["MS", "MA", "SA", "AA", "MM", "SS"]
DimensionLiteral = Literal[1, 2, 3]
ConductorPartRoleLiteral = Literal[
    "face_metal",
    "bump_body",
    "airbridge_post",
    "airbridge_deck",
]
ConductorRepresentationLiteral = Literal[
    "material_volume",
    "cutout_boundary_shell",
    "embedded_sheet",
    "sheet_with_construction_body",
]
ConductorUnionKindLiteral = Literal[
    "cad_fuse_same_material",
    "cad_contact_partition",
    "cutout_boundary_shell",
    "electrical_net",
]
PrimitiveRoleLiteral = Literal[
    "solution_volume",
    "construction_volume",
    "conductor_sheet",
    "imprint_surface",
    "imprint_curve",
    "ring_surface",
]
RingApplicationModeLiteral = Literal[
    "replace_parent_with_children",
    "overlay_postprocessing_only",
]
SurfaceParameterizationKindLiteral = Literal[
    "planar_uv",
    "cylindrical_uv",
    "occ_native_parametric",
]
SolverUseLiteral = Literal[
    "solver_active",
    "audit_only",
    "postprocessing_only",
]
Coordinate = tuple[float, float]
GmshDimTag = tuple[int, int]
BBox3D = tuple[float, float, float, float, float, float]
Vector3D = tuple[float, float, float]
PolygonRing = tuple[Coordinate, ...]


@dataclass(frozen=True)
class LayoutPolygonSpec:
    """Layout polygon identity before semantic route lowering."""

    polygon_id: str
    layer: str
    exterior: PolygonRing
    holes: tuple[PolygonRing, ...] = ()
    object_name: str | None = None
    net_name: str | None = None
    port_name: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SemanticEntitySpec:
    """Semantic geometry intent independent of temporary backend entity tags."""

    semantic_id: str
    role: str
    material_id: str
    priority: int
    geometry_kind: str
    part_role: ConductorPartRoleLiteral | None = None
    net_id: str | None = None
    polygon_ids: tuple[str, ...] = ()
    labels: tuple[str, ...] = ()
    host_void_semantic_id: str | None = None
    requires_construction_body: bool = False
    route_representations: Mapping[
        RouteLiteral,
        ConductorRepresentationLiteral,
    ] = field(default_factory=dict)
    geometry: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RoutePolicyRecord:
    """Route-level conductor union semantics, not a direct CAD operation."""

    route: RouteLiteral
    conductor_union_kind: ConductorUnionKindLiteral
    preserve_material_splits: bool = True
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class GeometryBuildInput:
    """Normalized adapter input for one semantic route geometry build."""

    polygons: tuple[LayoutPolygonSpec, ...]
    entities: tuple[SemanticEntitySpec, ...]
    route_policies: tuple[RoutePolicyRecord, ...] = ()
    solution_regions: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PrimitiveEntityRecord:
    """Primitive backend entity candidate before reference boolean topology."""

    primitive_id: str
    semantic_id: str
    dimension: DimensionLiteral
    role: PrimitiveRoleLiteral
    dim_tag: GmshDimTag | None = None
    source_polygon_ids: tuple[str, ...] = ()
    bbox: BBox3D | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class AtomicVolumeRecord:
    """Atomic reference volume after boolean partition and ownership."""

    atomic_id: str
    dim_tag: GmshDimTag | None = None
    covered_by_semantic_ids: tuple[str, ...] = ()
    primitive_ids: tuple[str, ...] = ()
    reference_owner_semantic_id: str | None = None
    bbox: BBox3D | None = None
    volume_um3: float | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class InterfacePatchRecord:
    """Reference shared-surface identity between semantic owners."""

    interface_id: str
    kind: InterfaceKindLiteral
    owner_semantic_ids: tuple[str, str]
    adjacent_atomic_volume_ids: tuple[str, str]
    dim_tag: GmshDimTag | None = None
    face_role: str | None = None
    normal_hint: Vector3D | None = None
    bbox: BBox3D | None = None
    area_um2: float | None = None
    solver_use: SolverUseLiteral = "solver_active"
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RingPatchRecord:
    """Inset ring plan for one interface patch; not final topology."""

    ring_id: str
    parent_interface_id: str
    label: str
    band_min_um: float
    band_max_um: float | None
    local_frame_id: str | None = None
    application_mode: RingApplicationModeLiteral = "replace_parent_with_children"
    parameterization: SurfaceParameterizationKindLiteral = "planar_uv"
    valid_routes: tuple[RouteLiteral, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CutHostOperationRecord:
    """Route operation: construction body cuts a final solution region."""

    operation_id: str
    construction_body_id: str
    host_solution_volume_id: str
    expected_exposed_boundary_kinds: tuple[str, ...] = ()
    remove_tool_entity_after_cut: bool = True
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SheetImprintOperationRecord:
    """Route operation: construction body footprint imprints a conductor sheet."""

    operation_id: str
    sheet_entity_id: str
    construction_body_id: str
    footprint_role: str
    keep_contact_cap_as_boundary: bool = False
    require_shared_edge_loop: bool = True
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RouteMaterializationRecord:
    """Route-specific final-geometry intent before final boolean topology."""

    route: RouteLiteral
    conductor_union_kind: ConductorUnionKindLiteral | None = None
    surviving_volume_ids: tuple[str, ...] = ()
    removed_volume_ids: tuple[str, ...] = ()
    reassigned_volume_owners: Mapping[str, str] = field(default_factory=dict)
    surviving_interface_ids: tuple[str, ...] = ()
    surviving_ring_ids: tuple[str, ...] = ()
    pec_boundary_interface_ids: tuple[str, ...] = ()
    construction_body_ids: tuple[str, ...] = ()
    cut_host_operations: tuple[CutHostOperationRecord, ...] = ()
    sheet_imprint_operations: tuple[SheetImprintOperationRecord, ...] = ()
    electrical_net_groups: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    boundary_shell_groups: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    destructive_operations: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class FinalTopologyRecord:
    """Audited live topology after route-specific final boolean build."""

    route: RouteLiteral
    live_volume_tags_by_semantic_id: Mapping[str, tuple[GmshDimTag, ...]] = field(
        default_factory=dict
    )
    live_surface_tags_by_source_id: Mapping[str, tuple[GmshDimTag, ...]] = field(
        default_factory=dict
    )
    removed_construction_body_ids: tuple[str, ...] = ()
    excluded_volume_ids: tuple[str, ...] = ()
    audit: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class FinalPhysicalGroupRecord:
    """Final physical-group plan emitted after final topology build."""

    physical_name: str
    dimension: int
    route: RouteLiteral
    role: str
    source_record_id: str
    net_id: str | None = None
    solver_use: SolverUseLiteral = "solver_active"
    entity_tags: tuple[int, ...] = ()
    logical_only: bool = False
    postprocessing_only: bool = False
    child_physical_names: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


class SemanticGeometryBuilder:
    """Internal facade for the semantic A/B/C route geometry pipeline."""

    def build(
        self,
        build_input: GeometryBuildInput,
        *,
        route: RouteLiteral,
    ) -> tuple[FinalPhysicalGroupRecord, ...]:
        """Build final physical-group record plans for one semantic route."""
        del build_input, route
        raise NotImplementedError("SemanticGeometryBuilder.build")


def normalize_geometry_inputs(build_input: GeometryBuildInput) -> GeometryBuildInput:
    """Validate caller-owned technology records and freeze semantic intent."""
    del build_input
    raise NotImplementedError("normalize_geometry_inputs")


def build_semantic_primitives(
    build_input: GeometryBuildInput,
) -> tuple[PrimitiveEntityRecord, ...]:
    """Create primitive backend entities from normalized semantic records."""
    del build_input
    raise NotImplementedError("build_semantic_primitives")


def normalize_primitives(
    primitives: Iterable[PrimitiveEntityRecord],
) -> tuple[PrimitiveEntityRecord, ...]:
    """Clean primitive entities without erasing semantic identity."""
    del primitives
    raise NotImplementedError("normalize_primitives")


def reference_boolean_topology_build(
    primitives: Iterable[PrimitiveEntityRecord],
) -> tuple[AtomicVolumeRecord, ...]:
    """Build reference boolean topology used for interface discovery."""
    del primitives
    raise NotImplementedError("reference_boolean_topology_build")


def resolve_semantic_ownership(
    atomic_volumes: Iterable[AtomicVolumeRecord],
    entities: Iterable[SemanticEntitySpec],
) -> tuple[AtomicVolumeRecord, ...]:
    """Choose the reference semantic owner for each atomic volume."""
    del atomic_volumes, entities
    raise NotImplementedError("resolve_semantic_ownership")


def build_interface_registry(
    atomic_volumes: Iterable[AtomicVolumeRecord],
) -> tuple[InterfacePatchRecord, ...]:
    """Build interface records from reference volume adjacency."""
    del atomic_volumes
    raise NotImplementedError("build_interface_registry")


def plan_inset_rings(
    interfaces: Iterable[InterfacePatchRecord],
    *,
    inset_margins_um: Iterable[float],
) -> tuple[RingPatchRecord, ...]:
    """Plan inset bands for selected interface patches."""
    del interfaces, inset_margins_um
    raise NotImplementedError("plan_inset_rings")


def materialize_route(
    atomic_volumes: Iterable[AtomicVolumeRecord],
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
    *,
    route: RouteLiteral,
) -> RouteMaterializationRecord:
    """Apply the selected A/B/C route policy to reference records."""
    del atomic_volumes, interfaces, rings, route
    raise NotImplementedError("materialize_route")


def final_boolean_topology_build(
    route_record: RouteMaterializationRecord,
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
) -> FinalTopologyRecord:
    """Build conformal final solver topology for the materialized route."""
    del route_record, interfaces, rings
    raise NotImplementedError("final_boolean_topology_build")


def export_physical_group_records(
    final_topology: FinalTopologyRecord,
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
) -> tuple[FinalPhysicalGroupRecord, ...]:
    """Export physical-group plans from audited final topology."""
    del final_topology, interfaces, rings
    raise NotImplementedError("export_physical_group_records")
