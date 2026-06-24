"""Semantic geometry build pipeline functions."""

from __future__ import annotations

from collections.abc import Iterable
from typing import get_args

from semantic_geometry_builder.models import (
    AtomicVolumeRecord,
    ConductorRepresentationLiteral,
    FinalPhysicalGroupRecord,
    FinalTopologyRecord,
    GeometryBuildInput,
    InterfacePatchRecord,
    PathInput,
    PrimitiveEntityRecord,
    RingPatchRecord,
    RouteLiteral,
    RouteMaterializationRecord,
    SemanticEntitySpec,
)
from semantic_geometry_builder.route_materialization import (
    build_route_materialization_context,
    materialize_route_a,
    materialize_route_b,
    materialize_route_c,
    validate_route_materialization,
)


class SemanticGeometryBuilder:
    """Facade for one semantic A/B/C route geometry compiler run.

    The builder treats `GeometryBuildInput` as frontend-normalized intent, then
    first constructs the fullest reference geometry needed to understand the
    device: solution regions, conductor sheets, construction bodies, bumps,
    airbridge posts/decks, and contact/imprint tools. This reference geometry is
    deliberately richer than the final solver geometry because it is used to
    discover ownership, adjacency, and real interface surfaces before any Route
    A/B/C simplification removes information.

    After the reference geometry is partitioned, the builder records semantic
    metadata for every meaningful topology decision: which semantic entities
    cover each atomic volume, which owner wins by priority, which surfaces are
    MS/MA/SA/MM/SS/AA interfaces, which interfaces are solver-active versus
    audit-only, and which inset rings should partition and replace parent
    surfaces. Backend dim-tags may help during this process, but the durable
    identity is always semantic ids, primitive ids, interface ids, ring ids, and
    operation ids.

    Route materialization then edits the geometry intent:

    - Route A is a mixed 2D/3D-boundary conductor representation. Face metals
      and attached airbridge decks become solver-active `surface_sheet`
      conductors, while Indium bumps and airbridge posts use construction
      bodies only to cut host regions and leave solver-facing PEC
      `cutout_boundary_shell` surfaces.
    - Route B is a cut-out boundary-shell conductor representation. Every
      conductor uses construction bodies to remove conductor interiors from
      solution regions, and the exposed PEC `cutout_boundary_shell` surfaces
      become the solver-facing conductor representation.
    - Route C is a retained material-volume conductor representation.
      Conductors remain as `material_volume` regions with conformal contact
      partitions, preserving material/semantic splits such as ground-plane to
      Indium-bump contacts instead of fusing them away.

    The final topology stage turns that route intent into conformal live
    topology and exports solver-neutral physical-group plans. The builder still
    does not write solver config or own mesh/report policy; it only emits the
    geometry/topology records and metadata sidecars that downstream consumers
    need.
    """

    def build(
        self,
        build_input: GeometryBuildInput,
        *,
        route: RouteLiteral,
        run_folder: PathInput,
    ) -> tuple[FinalPhysicalGroupRecord, ...]:
        """Run the full builder pipeline and emit final group record plans.

        `build_input` must already be produced by a frontend adapter. This
        method must not inspect GDSFactory, KQCircuits, or KLayout objects.

        `route` selects the final representation policy: Route A
        `surface_sheet` plus PEC `cutout_boundary_shell`, Route B cut-out PEC
        `cutout_boundary_shell`, or Route C retained `material_volume`.

        `run_folder` is the artifact root for this geometry compiler run. The
        builder owns JSON metadata under
        `run_folder/metadata/semantic_geometry/`, one reviewable sidecar per
        stage, plus optional backend snapshots referenced by those JSON files.

        Expected orchestration:
        validate input, build primitives, clean primitives, build reference
        topology, resolve ownership, build interfaces, plan rings, materialize
        the selected route, build final topology, then export physical-group
        records. The returned records are plans; downstream backend code may
        later attach real physical groups or generate solver config.
        """
        del build_input, route, run_folder
        raise NotImplementedError("SemanticGeometryBuilder.build")


def validate_geometry_input(build_input: GeometryBuildInput) -> GeometryBuildInput:
    """Validate a frontend-normalized GeometryBuildInput record.

    This is not frontend normalization. It checks the IR contract before any
    backend geometry exists: unique polygon/entity ids, valid polygon
    references, required solution regions such as `AIR`, construction-body host
    targets, `solution_regions` keys matching solution-domain semantic ids,
    valid route representations for each conductor part role, canonical material
    ids, unit metadata, and face/frame metadata for multi-die designs.

    Unknown layers, ambiguous materials, missing face mappings, priority ties,
    or invalid bump/post construction-body declarations should fail fast.

    Implementation note: verify that each `airbridge_deck` declares
    `attached_face_metal_semantic_id`, that the target is a `face_metal`, and
    that deck route representations follow the attached face-metal
    representations unless metadata documents an explicit override.
    """
    errors: list[str] = []
    polygon_ids: set[str] = set()
    entities_by_id: dict[str, SemanticEntitySpec] = {}

    for polygon in build_input.polygons:
        if not polygon.polygon_id:
            errors.append("polygon_id must be non-empty")
        elif polygon.polygon_id in polygon_ids:
            errors.append(f"duplicate polygon_id: {polygon.polygon_id}")
        polygon_ids.add(polygon.polygon_id)

    for entity in build_input.entities:
        if not entity.semantic_id:
            errors.append("semantic_id must be non-empty")
        elif entity.semantic_id in entities_by_id:
            errors.append(f"duplicate semantic_id: {entity.semantic_id}")
        entities_by_id[entity.semantic_id] = entity

        for polygon_id in entity.polygon_ids:
            if polygon_id not in polygon_ids:
                errors.append(
                    f"{entity.semantic_id} references unknown polygon_id: {polygon_id}"
                )

    solution_tokens = ("solution", "air", "substrate", "dielectric", "domain")
    solution_entity_ids = {
        entity.semantic_id
        for entity in build_input.entities
        if any(
            token in f"{entity.role} {entity.geometry_kind}".lower()
            for token in solution_tokens
        )
    }
    for semantic_id in build_input.solution_regions:
        if semantic_id not in solution_entity_ids:
            errors.append(
                f"solution_regions key must reference a solution-domain entity: "
                f"{semantic_id}"
            )

    valid_routes = set(get_args(RouteLiteral))
    valid_representations = set(get_args(ConductorRepresentationLiteral))
    for entity in build_input.entities:
        for route, representation in entity.route_representations.items():
            if route not in valid_routes:
                errors.append(f"{entity.semantic_id} has unsupported route: {route}")
            if representation not in valid_representations:
                errors.append(
                    f"{entity.semantic_id} has unsupported representation for "
                    f"{route}: {representation}"
                )

    for entity in build_input.entities:
        if entity.part_role != "airbridge_deck":
            continue

        target_id = entity.attached_face_metal_semantic_id
        if not target_id:
            errors.append(
                f"{entity.semantic_id} airbridge_deck requires "
                "attached_face_metal_semantic_id"
            )
            continue

        target = entities_by_id.get(target_id)
        if target is None:
            errors.append(
                f"{entity.semantic_id} attaches to unknown face metal: {target_id}"
            )
            continue
        if target.part_role != "face_metal":
            errors.append(
                f"{entity.semantic_id} attaches to non-face-metal entity: {target_id}"
            )
        if (
            not entity.metadata.get("route_inheritance_override_reason")
            and dict(entity.route_representations) != dict(target.route_representations)
        ):
            errors.append(
                f"{entity.semantic_id} route_representations must follow "
                f"attached face metal {target_id}"
            )

    if errors:
        raise ValueError("Invalid GeometryBuildInput: " + "; ".join(errors))

    return build_input


def build_semantic_primitives(
    build_input: GeometryBuildInput,
) -> tuple[PrimitiveEntityRecord, ...]:
    """Create backend primitive candidates from semantic records.

    This is the first stage that may create temporary backend handles such as
    Gmsh/OCC dim-tags. It builds route-independent primitive candidates required
    by declared semantic entities and route representations: solution-volume
    candidates, construction-volume candidates, conductor-sheet candidates, and
    imprint-surface or imprint-curve helpers. The selected route later decides
    which candidates survive, are removed, or are used only as construction
    tools.

    Each primitive must preserve `semantic_id`, source polygon ids, bbox, role,
    and any temporary backend handle in metadata. It must not fragment, resolve
    ownership, classify interfaces, create final topology, assign physical
    groups, mesh, or write solver config.
    """
    del build_input
    raise NotImplementedError("build_semantic_primitives")


def normalize_primitives(
    primitives: Iterable[PrimitiveEntityRecord],
) -> tuple[PrimitiveEntityRecord, ...]:
    """Clean primitive backend candidates without changing semantics.

    Allowed cleanup includes dropping empty primitives, caching bboxes,
    deduplicating exact duplicates when safe, healing local geometry defects,
    and fusing pieces only inside the same `semantic_id` when explicitly safe.

    Same material, same net, or same route is not enough reason to fuse. This
    stage must not cross semantic boundaries, lower A/B/C route policy, resolve
    ownership, repair final topology, or assign physical groups.

    Gmsh/OCC backend guidance: do not perform cross-semantic cutter grouping or
    temporary cutter union here. This stage may compute/cache bboxes that later
    enable Route A/B spatial batching, but temporary cutter grouping belongs to
    route materialization and final topology implementation, not primitive
    normalization.
    """
    del primitives
    raise NotImplementedError("normalize_primitives")


def reference_boolean_topology_build(
    primitives: Iterable[PrimitiveEntityRecord],
) -> tuple[AtomicVolumeRecord, ...]:
    """Build full construction topology for interface discovery.

    The reference topology keeps the highest-information construction geometry:
    solution volumes, construction bodies, sheets, and imprint tools are
    partitioned into atomic reference volumes. Atomic records preserve contributing
    primitive ids, covered semantic ids, temporary backend dim-tags, bboxes,
    optional volumes, and review metadata.

    This stage may write review snapshots referenced by metadata sidecars, but
    it must not apply Route A/B/C final decisions, remove conductor bodies,
    create inset rings, assign physical groups, or assume reference backend tags
    survive final topology.

    Gmsh/OCC backend guidance: preserve enough construction information to
    discover semantic ownership and interfaces. Avoid temporary cutter union in
    reference topology if it would erase per-bump, per-post, or per-airbridge
    provenance needed by the interface registry. For large designs, avoid one
    global all-to-all reference fragment when spatial filtering or local bbox
    batches can preserve `covered_by_semantic_ids`, primitive ids, resolved
    ownership, and MS/MA/SA/MM/SS/AA interface provenance. Enable
    `Geometry.OCCParallel = 1` around heavy OCC fragment/partition operations,
    not metadata indexing or simple primitive creation.
    """
    del primitives
    raise NotImplementedError("reference_boolean_topology_build")


def resolve_semantic_ownership(
    atomic_volumes: Iterable[AtomicVolumeRecord],
    entities: Iterable[SemanticEntitySpec],
) -> tuple[AtomicVolumeRecord, ...]:
    """Assign reference owners to atomic volumes from semantic coverage.

    Ownership is based on `covered_by_semantic_ids` and
    `SemanticEntitySpec.priority`, not material names or backend tags. The stage
    records `reference_owner_semantic_id`, preserves full coverage lineage, and
    fails on unresolved priority ties.

    This is still reference ownership. It must not apply route-specific
    reassignment, remove construction bodies, collapse same-net conductors,
    create interfaces, or build physical groups.
    """
    del atomic_volumes, entities
    raise NotImplementedError("resolve_semantic_ownership")


def build_interface_registry(
    atomic_volumes: Iterable[AtomicVolumeRecord],
) -> tuple[InterfacePatchRecord, ...]:
    """Discover semantic interface patches from reference adjacency.

    Adjacent reference-owned atomic volumes become `InterfacePatchRecord`s with
    stable interface ids, interface kind (`MS`, `MA`, `SA`, `MM`, `SS`, `AA`),
    owner semantic ids, adjacent atomic volume ids, face role, normal hint, bbox,
    area, and solver-use default.

    `MS`, `MA`, and `SA` default to solver-active. `MM`, `SS`, and `AA` remain
    valid topology/audit interfaces but default to audit-only. Classification
    must come from semantic adjacency, not after-the-fact surface names.
    """
    del atomic_volumes
    raise NotImplementedError("build_interface_registry")


def plan_inset_rings(
    interfaces: Iterable[InterfacePatchRecord],
    *,
    inset_margins_um: Iterable[float],
) -> tuple[RingPatchRecord, ...]:
    """Plan inset ring/core intent for selected interface patches.

    This creates `RingPatchRecord`s only. It records parent interface id,
    band min/max in micrometers, parameterization (`planar_uv`,
    `cylindrical_uv`, or `occ_native_parametric`), conformal partition mode,
    valid routes, and optional projected metadata.

    Default eligibility is `MS`, `MA`, and `SA`; `MM`, `SS`, and `AA` require an
    explicit debug/contact/postprocessing request. This stage must not split
    backend topology, create Gmsh surfaces, attach groups, create logical TOTAL
    rows, or create overlay ring surfaces/masks. Overlay is explicitly
    unsupported because it can create nonconformal geometry, duplicate boundary
    ownership, and ambiguous solver/EPR surface integration.
    """
    del interfaces, inset_margins_um
    raise NotImplementedError("plan_inset_rings")


def materialize_route(
    atomic_volumes: Iterable[AtomicVolumeRecord],
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
    *,
    route: RouteLiteral,
) -> RouteMaterializationRecord:
    """Lower reference records into route-specific final-geometry intent.

    This public function is only the dispatcher/integrator. It builds shared
    route context indexes, dispatches to `materialize_route_a`,
    `materialize_route_b`, or `materialize_route_c`, then runs common
    route-materialization validation.

    Route-specific files own the detailed A/B/C rules for surviving/removed
    volumes, ownership reassignments, surviving interfaces/rings, PEC boundary
    interfaces, construction bodies, cut-host operations, sheet-imprint
    operations, net groups, shell groups, and destructive-operation intent.

    Route A/B materialization should preserve per-body cut-host intent for
    repeated local blockers such as Indium bumps and airbridge posts. Compatible
    cutters may be batched later by final topology, but materialization must
    keep operation ids, construction-body ids, reference interfaces, exposed
    interface kinds, and exposed surface roles recoverable. Route C must not use
    temporary cutter union intent; it keeps `material_volume` regions and uses
    local contact partition/fragment logic so material and semantic splits
    survive.

    This stage must not execute backend booleans, create live final surfaces,
    assume CAD fuse is legal, attach groups, or emit solver config.

    Implementation note: enforce `airbridge_deck` route materialization from
    `attached_face_metal_semantic_id`; do not let decks silently pick an
    independent Route A/B/C representation unless metadata documents an
    explicit override.
    """
    context = build_route_materialization_context(
        atomic_volumes=atomic_volumes,
        interfaces=interfaces,
        rings=rings,
    )

    if route == "A":
        record = materialize_route_a(context)
    elif route == "B":
        record = materialize_route_b(context)
    elif route == "C":
        record = materialize_route_c(context)
    else:
        raise ValueError(f"Unsupported route: {route}")

    return validate_route_materialization(record, context)


def final_boolean_topology_build(
    route_record: RouteMaterializationRecord,
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
) -> FinalTopologyRecord:
    """Build conformal final topology from route materialization intent.

    This is where final backend topology is created. The stage executes host
    cuts, sheet imprints, shell extraction, ring/core partitioning, and final
    fragmentation so sheets, shells, rings, and solution domains are conformal.
    It then audits live backend tags and maps them back to stable semantic ids
    or namespaced surface source ids such as `IF__...`, `RING__...`, `OP__...`,
    or `PRIM__...`.

    Route A `surface_sheet` conductors must be imprinted/fragmented into the
    adjacent solution-domain topology as conformal PEC boundaries. Face-metal
    sheets should partition the relevant `SA` interface; suspended deck sheets
    may become conformal internal boundaries of `AIR`. They must not remain as
    disconnected standalone marker surfaces.

    Gmsh/OCC backend guidance: this is the main stage where Route A/B temporary
    cutter grouping should be applied. Group compatible construction bodies by
    host solution region, route, net when relevant, part role, expected exposed
    interface kinds, and exposed surface roles. The backend may pass many tools
    to one cut, form a temporary compound/fused cutter, or split work into
    spatial batches. This avoids slow per-object cut loops and avoids a fragile
    global fragment over all conductors and solution regions. The chosen
    strategy must preserve member provenance. After cutting, exposed shell
    surfaces must map back to stable source ids such as `IF__...`, `OP__...`,
    `PRIM__...`, or `RING__...` using reference interfaces, operation ids,
    construction body ids, bboxes, face roles, and spatial recovery. Do not
    apply this optimization to Route C material-volume contacts; Route C needs
    material/contact partitioning with material and semantic splits preserved.
    `Geometry.OCCParallel = 1` is appropriate around heavy cut, fragment,
    temporary fuse/compound, intersect, and duplicate-removal/coherence cleanup
    operations, but not semantic planning, record indexing, physical-name
    generation, metadata sidecar writing, or simple primitive construction.

    Required audits include: no Route A/B construction conductor volume remains
    in final solution-volume groups, conductor interiors are excluded from
    solution meshing, shell surfaces lie on final solution boundaries, contact
    caps are not duplicated overlays, MM/SS/AA audit entities are live, rings
    partition and replace their parent surface, parent ring surfaces become
    logical aggregates only, and Route C material splits survive when requested.

    This stage must not export solver config, attach material constants, treat
    removed construction bodies as solution volumes, or call final physical-group
    assignment unless a downstream backend assignment layer is explicitly used.
    """
    del route_record, interfaces, rings
    raise NotImplementedError("final_boolean_topology_build")


def export_physical_group_records(
    final_topology: FinalTopologyRecord,
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
) -> tuple[FinalPhysicalGroupRecord, ...]:
    """Export solver-neutral physical group plans from audited topology.

    The output records describe final solution volumes, solver-active boundary
    surfaces, audit/postprocessing interfaces, ring/core children, and logical
    aggregates such as TOTAL. They preserve `solver_use`, `logical_only`,
    `postprocessing_only`, source ids, child group names, and audited entity
    tags for a downstream backend/group writer.

    Parent interfaces replaced by ring children should become logical aggregates
    by default. `MM`, `SS`, and `AA` may appear in geometry manifests but remain
    audit-only unless explicitly requested. This stage must not create topology,
    repair topology, decide material constants, call backend physical-group APIs,
    or write solver config.
    """
    del final_topology, interfaces, rings
    raise NotImplementedError("export_physical_group_records")
