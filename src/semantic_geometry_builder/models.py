"""Semantic geometry contracts for route-aware bottom-up construction.

The v1 direction is topology-plan and tag-plan first:

`GeometryBuildInput -> 2D normalization -> planar arrangement -> stack
z-sweep/material occupancy -> InterfacePlan -> SurfacePartition/surface
candidates -> PointPlan -> CurvePlan -> SurfaceLoop -> canonical SurfacePlan
-> VolumePlan / ConstructionBodyPlan -> CutHostOperationPlan -> TagPlan`.

Backends must build every planned surface before they build any backend-live
volume. A volume is valid only after all of its boundary faces are represented
by `SurfacePlanRecord`s and referenced by `SurfaceRefRecord`s.

Topology-first canonical identity is part of the compiler contract. Shared
vertices, edges, and face patches must be planned as shared records before
backend lowering; a backend point/line cache is only an implementation detail,
not proof of conformal geometry. The OCC lowering contract is therefore:

`addPoint() -> addLine() -> addCurveLoop() -> addPlaneSurface()` for controlled
surfaces, then `addSurfaceLoop() -> addVolume()` for volumes.

Direct volume creation from boxes, extrusions, or unplanned bounds is not a v1
production path because it hides boundary surfaces from the semantic tag plan.
Global OCC fragment is not the semantic discovery mechanism.

Plan ids are the source of truth. Backend tags and physical-group ids are
lowering results mapped back to these records; the backend must not invent
semantic identity.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from os import PathLike
from typing import Any, Literal

PathInput = str | PathLike[str]
RUN_METADATA_DIR = "metadata"
SEMANTIC_GEOMETRY_METADATA_DIR = "semantic_geometry"

RouteLiteral = Literal["A", "B", "C"]
InterfaceKindLiteral = Literal["MS", "MA", "SA", "AA", "MM", "SS"]
DimensionLiteral = Literal[1, 2, 3]
SurfaceOrientationLiteral = Literal["forward", "reversed"]
Coordinate = tuple[float, float]
PolygonRing = tuple[Coordinate, ...]
Vector3D = tuple[float, float, float]
GmshDimTag = tuple[int, int]

ConductorPartRoleLiteral = Literal[
    "face_metal",
    "bump_body",
    "airbridge_post",
    "airbridge_deck",
]
HIGH_COUNT_LOCAL_CONDUCTOR_PART_ROLES = frozenset(
    ("bump_body", "airbridge_post", "airbridge_deck")
)
ConductorRepresentationLiteral = Literal[
    "material_volume",
    "cutout_boundary_shell",
    "surface_sheet",
]
ROUTE_ALLOWED_REPRESENTATIONS: dict[
    RouteLiteral,
    frozenset[ConductorRepresentationLiteral],
] = {
    "A": frozenset(("surface_sheet", "cutout_boundary_shell")),
    "B": frozenset(("cutout_boundary_shell",)),
    "C": frozenset(("material_volume",)),
}
SurfaceParameterizationKindLiteral = Literal[
    "planar_uv",
    "cylindrical_uv",
    "occ_native_parametric",
]
SurfacePartitionApplicationModeLiteral = Literal["replace_parent_with_children"]
InsetPartitionSourceLiteral = Literal[
    "coplanar_joint_arrangement",
    "surface_local_fallback",
]
CurveKindLiteral = Literal["line_segment"]
CurveOrientationLiteral = Literal[1, -1]
SurfaceLoopRoleLiteral = Literal["outer", "hole"]
TagSourceKindLiteral = Literal["surface", "volume"]
SolverUseLiteral = Literal["solver_active"]
DEFAULT_INTERFACE_SOLVER_USE: dict[InterfaceKindLiteral, SolverUseLiteral] = {
    "MS": "solver_active",
    "MA": "solver_active",
    "SA": "solver_active",
    "AA": "solver_active",
    "MM": "solver_active",
    "SS": "solver_active",
}


@dataclass(frozen=True)
class LayoutPolygonSpec:
    """Adapter-normalized polygon with stable frontend provenance."""

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
    """Stable semantic object before route-aware construction planning."""

    semantic_id: str
    role: str
    material_id: str
    priority: int
    geometry_kind: str
    part_role: ConductorPartRoleLiteral | None = None
    attached_face_metal_semantic_id: str | None = None
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
class GeometryBuildInput:
    """Adapter boundary for route-aware semantic geometry construction."""

    polygons: tuple[LayoutPolygonSpec, ...]
    entities: tuple[SemanticEntitySpec, ...]
    solution_regions: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class InterfacePlanRecord:
    """Pre-construction declaration that two semantic owners share a surface.

    `interface_id` is the stable semantic id and must start with the interface
    kind directly, such as `MM__`, `MS__`, `MA__`, `SA__`, `SS__`, or `AA__`.
    There is no extra `IF__` prefix.

    Every interface must be recognized before OCC geometry creation. This
    record only states the semantic fact: which two owners meet, what kind of
    interface it is, and which recognition rule proved it. Backend-live
    geometry is created only from `SurfacePlanRecord`.

    Required metadata for bottom-up OCC planning should be attached here as
    soon as it is recognized: `contact_plane` for the physical plane of contact,
    `footprint` for projected contact geometry, and `loop_geometry_ref` when the
    interface is already known as an explicit loop source.

    Route A surface sheets may need side-aware semantics on the same geometric
    loop. Use metadata such as `surface_side`, `adjacent_domain_id`,
    `excluded_owner_semantic_id`, and `interface_kinds` to say which side is
    MA/MS/MM without creating overlapping backend surfaces.
    """

    interface_id: str
    kind: InterfaceKindLiteral
    owner_semantic_ids: tuple[str, str]
    recognition_rule: str
    source_polygon_ids: tuple[str, ...] = ()
    host_domain_id: str | None = None
    solver_use: SolverUseLiteral | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.interface_id.startswith(f"{self.kind}__"):
            raise ValueError(
                f"interface_id {self.interface_id!r} must start with "
                f"{self.kind}__"
            )
        if self.solver_use is None:
            object.__setattr__(
                self,
                "solver_use",
                DEFAULT_INTERFACE_SOLVER_USE[self.kind],
            )


@dataclass(frozen=True)
class SurfacePlanRecord:
    """One backend-live planned surface to create before volume assembly.

    Shared interfaces are represented by one `surface_id`, and both owning
    volumes must reference that same id. Partitioned interfaces, including
    inset rings, are already expanded into child `SurfacePlanRecord`s before
    backend lowering. The backend must create each child surface directly from
    its loops and hole loops; it must not cut a parent surface later.

    v1 backend lowering should consume `outer_loop_ref` and `hole_loop_refs`.
    `geometry_ref` remains as audit/source metadata: it may carry `plane` or
    `contact_plane`, `outer_loop`, optional `hole_loops`, optional `footprint`,
    optional `inset_band`, and optional `loop_geometry_ref`. Inset children that
    belong to the same coplanar arrangement should also carry
    `inset_family_ids` or `coplanar_partition_id` so reviewers can see that
    `SA`/`MS`/`MA` children were generated from one shared plane graph instead
    of from isolated per-surface offsets. A surface that cannot be tied to
    canonical loop refs must fail during canonicalization rather than falling
    back to global fragment-based discovery.

    `metadata` may carry side/exposure semantics such as `interface_kinds`,
    `surface_side`, `owner_semantic_ids`, `adjacent_domain_id`,
    and `excluded_owner_semantic_id`. Route A surface-sheet conductors must be
    represented by these interface surfaces, not by extra standalone sheet
    surfaces.
    When a surface belongs to more than one geometry interface kind, physical
    names must include the joined interface kinds, for example `MM_MS__...`.
    """

    surface_id: str
    owner_semantic_id: str
    surface_role: str
    geometry_ref: Mapping[str, Any]
    outer_loop_ref: str | None = None
    hole_loop_refs: tuple[str, ...] = ()
    interface_id: str | None = None
    parent_surface_id: str | None = None
    partition_label: str | None = None
    normal_hint: Vector3D | None = None
    valid_routes: tuple[RouteLiteral, ...] = ()
    solver_use: SolverUseLiteral = "solver_active"
    construction_only: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SurfaceRefRecord:
    """Reference to one planned surface from a volume surface loop."""

    surface_id: str
    orientation: SurfaceOrientationLiteral
    role: str
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PointPlanRecord:
    """One canonical planned point shared by all touching curves.

    A conformal v1 plan may not contain two live point records at the same
    coordinate. Points are topology/audit records, not physical groups.
    """

    point_id: str
    coordinate: Vector3D
    owner_semantic_ids: tuple[str, ...] = ()
    interface_ids: tuple[str, ...] = ()
    used_by_curve_ids: tuple[str, ...] = ()
    boundary_volume_ids: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CurvePlanRecord:
    """One canonical planned curve shared by all touching surfaces.

    Curves are topology/audit/mesh-refinement contract records, not physical
    groups. `used_by_surface_ids`, `interface_ids`, and `boundary_volume_ids`
    describe how the curve participates in the final topology without forcing a
    single semantic owner for shared edges.
    """

    curve_id: str
    curve_kind: CurveKindLiteral
    start_point_id: str
    end_point_id: str
    owner_semantic_ids: tuple[str, ...] = ()
    interface_ids: tuple[str, ...] = ()
    used_by_surface_ids: tuple[str, ...] = ()
    boundary_volume_ids: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CurveRefRecord:
    """Oriented reference to one canonical curve from a surface loop."""

    curve_id: str
    orientation: CurveOrientationLiteral
    role: str
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SurfaceLoopRecord:
    """Canonical loop used by a planned surface.

    A surface owns one outer loop and zero or more hole loops by reference.
    The loop owns ordered `CurveRefRecord`s so shared boundaries are explicit
    before OCC lowering.
    """

    loop_id: str
    curve_refs: tuple[CurveRefRecord, ...]
    role: SurfaceLoopRoleLiteral
    surface_id: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class VolumePlanRecord:
    """One planned volume assembled from already-planned surfaces.

    Backend-live volumes must be created from `surface_refs`: the planner first
    controls every boundary surface, and the backend then calls
    `addSurfaceLoop()` plus `addVolume()` on those surfaces. `metadata` may keep
    domain bounds, material, or stack provenance for audit, but those records are
    not permission to create a box or extruded fallback volume.

    Route C material volumes are backend-live solver geometry. Route A/B
    construction cutters are not represented here; they use
    `ConstructionBodyPlanRecord` so retained material volumes and temporary
    cutter bodies cannot be confused.
    """

    volume_id: str
    owner_semantic_id: str
    material_id: str
    surface_refs: tuple[SurfaceRefRecord, ...]
    valid_routes: tuple[RouteLiteral, ...] = ()
    construction_only: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ConstructionBodyPlanRecord:
    """Route A/B temporary body used to derive final cutout shell surfaces.

    This is not final solver geometry and must not receive a physical group.
    It records which semantic conductor blocks a host solution region and which
    final shell surfaces are expected to survive after the backend cut.
    """

    construction_body_id: str
    owner_semantic_id: str
    host_semantic_id: str
    representation: ConductorRepresentationLiteral
    geometry_ref: Mapping[str, Any]
    expected_surface_ids: tuple[str, ...] = ()
    valid_routes: tuple[RouteLiteral, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CutHostOperationRecord:
    """Route A/B operation that cuts one host with construction bodies.

    Compatible operation records may later be batched by the backend, but the
    semantic operation identity and member construction bodies must remain
    recoverable for provenance.
    """

    operation_id: str
    host_semantic_id: str
    construction_body_ids: tuple[str, ...]
    exposed_surface_ids: tuple[str, ...]
    valid_routes: tuple[RouteLiteral, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class TagPlanRecord:
    """Solver-neutral physical tag plan before backend entity tags exist.

    Tags are planned before OCC construction. `source_record_kind` and
    `source_record_id` must point to a live `SurfacePlanRecord` or
    `VolumePlanRecord`. After backend construction, the same source id is used
    to recover OCC dim-tags and create final physical groups. Together with
    `BackendEntityTagRecord`, this is the tag ledger: plan ids are stable,
    OCC tags are lowering results, and physical names are solver-facing labels.
    """

    physical_name: str
    dimension: DimensionLiteral
    source_record_kind: TagSourceKindLiteral
    source_record_id: str
    role: str
    solver_use: SolverUseLiteral = "solver_active"
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        expected_dimension = 2 if self.source_record_kind == "surface" else 3
        if self.dimension != expected_dimension:
            raise ValueError(
                f"{self.source_record_kind} tag {self.physical_name!r} "
                f"must use dimension {expected_dimension}"
            )


@dataclass(frozen=True)
class BackendEntityTagRecord:
    """Backend dim-tag recovered for one live planned source record.

    Two live source ids mapping to the same backend dim-tag means
    canonicalization failed earlier; do not accept that as a backend shortcut.
    """

    source_record_kind: TagSourceKindLiteral
    source_record_id: str
    dim_tag: GmshDimTag
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SurfacePartitionRecord:
    """Partition intent for one child surface of a parent interface.

    This is not a backend geometry object and is not a physical group source.
    It tells the planner why a parent interface, such as `MS__Metal__Substrate`,
    is represented by child live surfaces such as
    `SURF__MS__Metal__Substrate__RING_0_50NM` and
    `SURF__MS__Metal__Substrate__CORE`.

    `metadata` may carry `inset_band`, `outer_loop`, `hole_loops`,
    `loop_geometry_ref`, `inset_family_ids`, and `coplanar_partition_id` for the
    child surface. For coplanar `SA`/`MS`/`MA` families, this record is not
    proof that one isolated parent was safely offset; it must point back to the
    shared plane arrangement that generated every adjacent child surface on
    that plane. The child must be directly buildable; overlay masks are not
    supported.
    """

    partition_id: str
    parent_interface_id: str
    child_surface_id: str
    label: str
    band_min_um: float | None = None
    band_max_um: float | None = None
    application_mode: SurfacePartitionApplicationModeLiteral = (
        "replace_parent_with_children"
    )
    parameterization: SurfaceParameterizationKindLiteral = "planar_uv"
    valid_routes: tuple[RouteLiteral, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CoplanarInsetFamilyRecord:
    """Topology contract for inset children that live on one shared plane.

    A family is not a comment on individual surfaces; it is the reviewable
    statement that several parent interfaces share one plane and one boundary
    volume, so their inset children must be generated from one coplanar
    arrangement. `surface_local_fallback` is allowed only for isolated parents
    and must fail fast when adjacent parents would otherwise create independent
    near-duplicate points or curves.
    """

    family_id: str
    plane_key: str
    boundary_volume_id: str
    parent_surface_ids: tuple[str, ...]
    breakpoints_um: tuple[float, ...]
    source: InsetPartitionSourceLiteral
    child_surface_ids: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MeshSizeHintRecord:
    """Mesh-size contract derived from planned topology, not from backend mesh.

    Inset bands are solver-active geometry, so the compiler must tell downstream
    mesh adapters the smallest local feature it created. A 50 nm band should
    produce a max-size hint near 25 nm, otherwise CAD can be conformal while the
    generated tetrahedra are still unsafe for solver use.
    """

    target_id: str
    max_size_um: float
    reason: str
    source_partition_id: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ConstructionPlanRecord:
    """Route-aware handoff from semantic planning to bottom-up OCC creation.

    This is the complete input to the backend. `interfaces` explain why shared
    surfaces exist, `surface_partitions` explain parent-to-child inset
    coverage, `coplanar_inset_families` explain shared-plane inset ownership,
    `mesh_size_hints` explain solver-mesh constraints created by tiny planned
    features, `points`, `curves`, and `surface_loops` make shared topology
    canonical, `surfaces` and `volumes` describe geometry to build,
    `construction_bodies` and `cut_operations` describe Route A/B host cuts,
    and `tags` define physical names before backend dim-tags exist.

    After OCC construction, the backend returns the same plan with
    `backend_entity_tags` populated. `FinalPhysicalGroupRecord`s are then a
    deterministic projection of `tags + backend_entity_tags`.
    """

    route: RouteLiteral
    interfaces: tuple[InterfacePlanRecord, ...] = ()
    surface_partitions: tuple[SurfacePartitionRecord, ...] = ()
    coplanar_inset_families: tuple[CoplanarInsetFamilyRecord, ...] = ()
    mesh_size_hints: tuple[MeshSizeHintRecord, ...] = ()
    points: tuple[PointPlanRecord, ...] = ()
    curves: tuple[CurvePlanRecord, ...] = ()
    surface_loops: tuple[SurfaceLoopRecord, ...] = ()
    surfaces: tuple[SurfacePlanRecord, ...] = ()
    volumes: tuple[VolumePlanRecord, ...] = ()
    construction_bodies: tuple[ConstructionBodyPlanRecord, ...] = ()
    cut_operations: tuple[CutHostOperationRecord, ...] = ()
    tags: tuple[TagPlanRecord, ...] = ()
    backend_entity_tags: tuple[BackendEntityTagRecord, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class FinalPhysicalGroupRecord:
    """Final solver-neutral group plan after backend tags are known."""

    physical_name: str
    dimension: int
    route: RouteLiteral
    role: str
    source_record_id: str
    net_id: str | None = None
    solver_use: SolverUseLiteral | None = None
    entity_tags: tuple[int, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.solver_use is None:
            object.__setattr__(self, "solver_use", "solver_active")
        if not self.entity_tags:
            raise ValueError("physical groups require backend entity tags")
