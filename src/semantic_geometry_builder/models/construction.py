"""Construction-plan records.

These records explain how route-specific surfaces, construction bodies, mesh
hints, and the aggregate handoff plan are assembled before backend lowering.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from semantic_geometry_builder.models.common import (
    ConductorRepresentationLiteral,
    InsetPartitionSourceLiteral,
    RouteLiteral,
    SurfaceParameterizationKindLiteral,
    SurfacePartitionApplicationModeLiteral,
)
from semantic_geometry_builder.models.regions import PortSheetRegionRecord
from semantic_geometry_builder.models.tags import BackendEntityTagRecord, TagPlanRecord
from semantic_geometry_builder.models.topology import (
    CurvePlanRecord,
    InterfacePlanRecord,
    PointPlanRecord,
    SurfaceLoopRecord,
    SurfacePlanRecord,
    VolumePlanRecord,
)


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
class ConstructionPlanRecord:
    """Route-aware handoff from semantic planning to bottom-up OCC creation.

    This is the complete input to the backend. `interfaces` explain why shared
    surfaces exist, `surface_partitions` explain parent-to-child inset
    coverage, `coplanar_inset_families` explain shared-plane inset ownership,
    `mesh_size_hints` explain solver-mesh constraints created by tiny planned
    features, `points`, `curves`, and `surface_loops` make shared topology
    canonical, `surfaces` and `volumes` describe geometry to build,
    `construction_bodies` and `cut_operations` describe Route A/B host cuts,
    `port_sheet_regions` carry explicit 2D lumped-port overlap intent, and
    `tags` define physical names before backend dim-tags exist.

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
    port_sheet_regions: tuple[PortSheetRegionRecord, ...] = ()
