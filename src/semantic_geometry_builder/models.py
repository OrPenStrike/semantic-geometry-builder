"""Semantic geometry data contracts.

Models carry stable semantic identity and solver-neutral topology intent.
Backend tags such as Gmsh/OCC dim-tags are transient provenance handles, not
the source of truth.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from os import PathLike
from typing import Any, Literal

PathInput = str | PathLike[str]
RUN_METADATA_DIR = "metadata"
SEMANTIC_GEOMETRY_METADATA_DIR = "semantic_geometry"
# Route selection is the top-level representation family for one build:
#
# - A: mixed surface-sheet / PEC-shell representation. Face metal and
#   airbridge decks lower to solver-active `surface_sheet` conductors; Indium
#   bumps and airbridge posts use construction bodies to cut hosts, then expose
#   PEC `cutout_boundary_shell` surfaces.
# - B: cut-out boundary-shell representation. All conductors block fields by
#   cutting solution regions; final conductor volumes are removed and PEC
#   `cutout_boundary_shell` surfaces survive.
# - C: retained material-volume representation. Conductor `material_volume`
#   regions survive and material contact splits are preserved.
RouteLiteral = Literal["A", "B", "C"]
# Interface kind is semantic adjacency, not a physical-name formatting detail:
#
# - MS/MA/SA are the default solver-active dielectric or conductor boundaries.
# - MM/SS/AA are valid topology/contact/audit interfaces. They should survive in
#   manifests when useful, but default to audit-only solver use.
InterfaceKindLiteral = Literal["MS", "MA", "SA", "AA", "MM", "SS"]
DimensionLiteral = Literal[1, 2, 3]
# Conductor part role drives route policy decisions. It is more specific than a
# generic `role="conductor"`:
#
# - face_metal: planar metal that may be a Route A surface sheet.
# - bump_body: 3D conductor that normally needs construction-body cuts/imprints.
# - airbridge_post: bump-like 3D support/contact conductor.
# - airbridge_deck: follows `attached_face_metal_semantic_id` route
#   representation by default.
ConductorPartRoleLiteral = Literal[
    "face_metal",
    "bump_body",
    "airbridge_post",
    "airbridge_deck",
]
# Final conductor representation requested by route policy:
#
# - material_volume: retained conductor volume, Route C style.
# - cutout_boundary_shell: final PEC boundary surfaces after construction-body
#   cuts, used by Route B conductor shells and Route A bump/post shells.
# - surface_sheet: solver-active 2D conductor boundary, Route A face metal/deck
#   style. This is not a passive embedded marker inside AIR.
ConductorRepresentationLiteral = Literal[
    "material_volume",
    "cutout_boundary_shell",
    "surface_sheet",
]
# Route-level conductor union policy. These values describe semantic lowering,
# not direct backend operations such as one Gmsh fuse call:
#
# - cad_fuse_same_material:
#   Safe only when pieces share material and semantic identity, for example
#   pieces of one face-metal object. Do not use for Al/Nb-to-Indium contacts
#   where material/semantic splits must survive.
# - cad_contact_partition:
#   Route C material-volume policy. Volumes survive and contacts are conformal,
#   but material/semantic splits are preserved by partition/fragment instead of
#   CAD fuse, for example ground plane to Indium bump.
# - cutout_boundary_shell:
#   Route B policy. Conductor construction bodies cut solution regions, final
#   conductor volume is removed, and exposed PEC shells survive as boundaries.
# - electrical_net:
#   Route A policy. Sheets/shells belong to one electrical net such as GND/PEC,
#   but geometry stays split, for example face-metal `surface_sheet`, bump/post
#   PEC shell, and airbridge-deck `surface_sheet`.
ConductorUnionKindLiteral = Literal[
    "cad_fuse_same_material",
    "cad_contact_partition",
    "cutout_boundary_shell",
    "electrical_net",
]
# Primitive role describes backend construction candidates before reference
# boolean topology. These are not final physical groups:
#
# - solution_volume: AIR/substrate/dielectric domains.
# - construction_volume: temporary or retained 3D bodies used for cuts,
#   partitions, or material-volume routes.
# - conductor_sheet: 2D conductor boundary candidate.
# - imprint_surface/imprint_curve: topology-splitting tools for contacts.
# - ring_surface: reserved for explicit ring topology when a backend stage
#   creates it; ring planning itself uses RingPatchRecord.
PrimitiveRoleLiteral = Literal[
    "solution_volume",
    "construction_volume",
    "conductor_sheet",
    "imprint_surface",
    "imprint_curve",
    "ring_surface",
]
# Ring application mode is intentionally single-mode for final geometry:
# inset rings must partition the parent interface, and child ring/core surfaces
# replace the parent as live physical surfaces. Do not model overlay ring
# surfaces or overlay masks here: they can make geometry nonconformal, duplicate
# boundary ownership, and confuse solver/EPR surface integration.
RingApplicationModeLiteral = Literal["replace_parent_with_children"]
# Surface parameterization tells final topology how to construct inset rings:
#
# - planar_uv: flat interfaces such as face-metal MA/MS/SA.
# - cylindrical_uv: sidewalls such as bumps/posts.
# - occ_native_parametric: fallback when local planar/cylindrical assumptions do
#   not describe the surface safely.
SurfaceParameterizationKindLiteral = Literal[
    "planar_uv",
    "cylindrical_uv",
    "occ_native_parametric",
]
# Solver use is a downstream-consumption hint, not topology ownership:
#
# - solver_active: safe default for surfaces that a solver config may consume.
# - audit_only: geometry/contact/provenance exists but should not become a
#   solver boundary by default.
# - postprocessing_only: emitted for analysis/reporting, not solver setup.
SolverUseLiteral = Literal[
    "solver_active",
    "audit_only",
    "postprocessing_only",
]
DEFAULT_INTERFACE_SOLVER_USE: dict[InterfaceKindLiteral, SolverUseLiteral] = {
    "MS": "solver_active",
    "MA": "solver_active",
    "SA": "solver_active",
    "AA": "audit_only",
    "MM": "audit_only",
    "SS": "audit_only",
}
Coordinate = tuple[float, float]
GmshDimTag = tuple[int, int]
BBox3D = tuple[float, float, float, float, float, float]
Vector3D = tuple[float, float, float]
PolygonRing = tuple[Coordinate, ...]


@dataclass(frozen=True)
class LayoutPolygonSpec:
    """Adapter-normalized layout polygon before semantic route lowering.

    `layer` and optional object/net/port names preserve frontend provenance.
    They do not decide final material ownership by themselves; semantic entities
    and route policies do that.
    """

    polygon_id: str
    layer: str
    exterior: PolygonRing
    holes: tuple[PolygonRing, ...] = ()
    object_name: str | None = None
    net_name: str | None = None
    port_name: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


# SemanticEntitySpec is the durable frontend-to-builder identity record. It is
# where adapter-normalized layer/material/net/part-role meaning enters the IR.
# Later backend tags, booleans, and physical groups must point back here rather
# than rediscovering meaning from geometry.
@dataclass(frozen=True)
class SemanticEntitySpec:
    """Stable semantic geometry intent independent of backend entity tags.

    `semantic_id` is the durable identity used across primitive creation,
    reference topology, route materialization, and final physical-group export.
    Use `AIR` for air or vacuum-like solution regions.

    For `part_role == "airbridge_deck"`, `attached_face_metal_semantic_id`
    identifies the face-metal entity whose route representation the deck should
    inherit. If a deck intentionally diverges, metadata must document an
    explicit route-inheritance override reason.
    """

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


# RoutePolicyRecord captures high-level route lowering choices for the whole
# build or a route slice. It is intentionally semantic policy, so downstream
# materialization can decide which backend operations are legal.
@dataclass(frozen=True)
class RoutePolicyRecord:
    """Route-level conductor union semantics, not a direct CAD operation."""

    route: RouteLiteral
    conductor_union_kind: ConductorUnionKindLiteral
    preserve_material_splits: bool = True
    metadata: Mapping[str, Any] = field(default_factory=dict)


# GeometryBuildInput is the adapter boundary. GDSFactory, KQCircuits, or raw
# KLayout code must lower into this record before the semantic builder starts;
# the builder should not inspect frontend objects directly.
@dataclass(frozen=True)
class GeometryBuildInput:
    """Normalized adapter input for one semantic route geometry build.

    Frontends such as GDSFactory, KQCircuits, or raw KLayout adapters should
    lower into this record instead of passing frontend objects into the builder.

    `entities` defines stable semantic identity, material ownership, role,
    priority, and route behavior. `solution_regions` maps solution-domain
    semantic ids to construction metadata such as AIR/substrate/dielectric
    boxes, z-ranges, bounding boxes, or domain-specific geometry parameters.
    Each key should normally match a `SemanticEntitySpec.semantic_id` whose
    role is a solver-domain region unless the adapter documents otherwise.
    """

    polygons: tuple[LayoutPolygonSpec, ...]
    entities: tuple[SemanticEntitySpec, ...]
    route_policies: tuple[RoutePolicyRecord, ...] = ()
    solution_regions: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


# PrimitiveEntityRecord is the first bridge from semantic IR into backend
# geometry handles. `dim_tag` is allowed here only as temporary provenance; the
# primitive id and semantic id remain the stable identity.
@dataclass(frozen=True)
class PrimitiveEntityRecord:
    """Primitive backend entity candidate before reference boolean topology.

    `dim_tag` may carry a temporary backend handle for audit/debugging. The
    stable link back to intent is `semantic_id` plus `source_polygon_ids`.
    """

    primitive_id: str
    semantic_id: str
    dimension: DimensionLiteral
    role: PrimitiveRoleLiteral
    dim_tag: GmshDimTag | None = None
    source_polygon_ids: tuple[str, ...] = ()
    bbox: BBox3D | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


# AtomicVolumeRecord represents a reference/full-construction volume after
# boolean partition. Ownership may still be unresolved until
# resolve_semantic_ownership() fills reference_owner_semantic_id.
@dataclass(frozen=True)
class AtomicVolumeRecord:
    """Atomic reference volume after boolean partition.

    Atomic volumes are reference records used to discover ownership and
    adjacency. `reference_owner_semantic_id` may remain `None` until
    `resolve_semantic_ownership()` assigns it. Route materialization may later
    retain, remove, or reassign the resolved volumes.
    """

    atomic_id: str
    dim_tag: GmshDimTag | None = None
    covered_by_semantic_ids: tuple[str, ...] = ()
    primitive_ids: tuple[str, ...] = ()
    reference_owner_semantic_id: str | None = None
    bbox: BBox3D | None = None
    volume_um3: float | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


# InterfacePatchRecord is the semantic shared-surface record. It keeps both
# owners and adjacent atomic volumes so solver-active boundaries, contact/audit
# interfaces, and ring planning all trace back to reference adjacency.
@dataclass(frozen=True)
class InterfacePatchRecord:
    """Reference shared-surface identity between semantic owners.

    `MS`, `MA`, and `SA` default to solver-active. `MM`, `SS`, and `AA` are
    valid topology/contact/audit interfaces and default to `audit_only`.
    """

    interface_id: str
    kind: InterfaceKindLiteral
    owner_semantic_ids: tuple[str, str]
    adjacent_atomic_volume_ids: tuple[str, str]
    dim_tag: GmshDimTag | None = None
    face_role: str | None = None
    normal_hint: Vector3D | None = None
    bbox: BBox3D | None = None
    area_um2: float | None = None
    solver_use: SolverUseLiteral | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.solver_use is None:
            object.__setattr__(
                self,
                "solver_use",
                DEFAULT_INTERFACE_SOLVER_USE[self.kind],
            )


# RingPatchRecord is planned inset intent, not topology. It tells final topology
# which parent interface should be partitioned into child bands/core and how.
# The child surfaces replace the parent in final geometry; the parent can only
# survive as a logical aggregate with no backend entity tags.
@dataclass(frozen=True)
class RingPatchRecord:
    """Inset ring/core intent for one interface patch; not final topology.

    Labels may describe bands such as `BAND_0_50NM` or `CORE_AFTER_1UM`.
    Conformal backend topology is created later by `final_boolean_topology_build`.
    Overlay ring surfaces or masks are explicitly unsupported because they can
    create nonconformal or overlapping parent/child boundary ownership.
    """

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


# CutHostOperationRecord is route materialization intent for construction-body
# subtraction. It records what should cut which solution region before backend
# booleans happen. Backends may batch compatible records for performance, but
# the records stay per-construction-body so provenance is not lost.
@dataclass(frozen=True)
class CutHostOperationRecord:
    """Route operation: construction body cuts a final solution region.

    `expected_exposed_interface_kinds` records semantic interface kinds expected
    after the cut, such as `MA`. `expected_exposed_surface_roles` records
    geometric or part-specific roles, such as `sidewall`, `post_sidewall`,
    `deck_top`, `deck_bottom`, or `deck_sidewall`.

    Gmsh/OCC backend guidance: this is intentionally per-construction-body
    semantic cut intent. Route A/B backends may group compatible records into a
    temporary cutter batch before boolean execution, but that grouping is only a
    performance optimization. It must preserve member provenance through
    operation ids, construction body ids, reference interfaces, and spatial
    recovery metadata. Do not treat temporary cutter grouping as final
    CAD/material union; Route C material volumes and dissimilar contacts must
    preserve material and semantic splits.
    """

    operation_id: str
    construction_body_id: str
    host_solution_volume_id: str
    expected_exposed_interface_kinds: tuple[InterfaceKindLiteral, ...] = ()
    expected_exposed_surface_roles: tuple[str, ...] = ()
    remove_tool_entity_after_cut: bool = True
    metadata: Mapping[str, Any] = field(default_factory=dict)


# SheetImprintOperationRecord is route materialization intent for conformal
# sheet/contact splitting, for example bump footprints or airbridge post feet.
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


# RouteMaterializationRecord is the handoff from reference topology into final
# topology build. It says what should survive, be removed, be reassigned, cut,
# imprinted, grouped by net, or exposed as boundary shell; it still does not
# execute backend booleans or create physical groups.
@dataclass(frozen=True)
class RouteMaterializationRecord:
    """Route-specific final-geometry intent before final boolean topology.

    Route A, B, and C choices are represented here as surviving, removed,
    reassigned, and boundary-shell records. This record still describes intent;
    it is not the conformal backend topology.
    """

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


# FinalTopologyRecord is the audited live backend topology map after final
# route-specific booleans/fragments. Backend tags may appear here for consumers,
# but the keys remain semantic/source ids.
@dataclass(frozen=True)
class FinalTopologyRecord:
    """Audited live topology after route-specific final boolean build.

    Live tags are emitted for solver adapters, but semantic/source ids remain
    the keys. `live_surface_tags_by_source_id` keys must be namespaced to avoid
    collisions across source record types, for example `IF__...`, `RING__...`,
    `OP__...`, or `PRIM__...`. Audit metadata records the checks that made the
    topology usable.
    """

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


# FinalPhysicalGroupRecord is a solver-neutral group plan. It can describe real
# backend entity tags or logical aggregates such as TOTAL; actual backend
# physical-group assignment and solver config generation happen downstream.
@dataclass(frozen=True)
class FinalPhysicalGroupRecord:
    """Final physical-group plan emitted after final topology build.

    This is solver-neutral. Logical aggregates such as `TOTAL`, `TOTAL_MA`, or
    `IF__...__TOTAL` must use `logical_only=True` and reference
    `child_physical_names` instead of claiming backend entity tags. Use
    `role="logical_total"` for logical total aggregates even when the physical
    name does not follow one of those examples.
    """

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

    def __post_init__(self) -> None:
        logical_total_name = (
            self.physical_name == "TOTAL"
            or self.physical_name.startswith("TOTAL_")
            or self.physical_name.endswith("__TOTAL")
        )
        if (
            self.role == "logical_total" or logical_total_name
        ) and not self.logical_only:
            raise ValueError("logical total groups must be logical_only")
        if self.logical_only and self.entity_tags:
            raise ValueError(
                "logical-only physical groups must not carry backend entity tags"
            )
