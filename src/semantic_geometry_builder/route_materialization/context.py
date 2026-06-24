"""Shared route materialization context."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from semantic_geometry_builder.models import (
    DEFAULT_INTERFACE_SOLVER_USE,
    AtomicVolumeRecord,
    InterfaceKindLiteral,
    InterfacePatchRecord,
    RingPatchRecord,
)


@dataclass(frozen=True)
class RouteMaterializationContext:
    """Indexed reference records shared by route-specific materializers.

    These indexes must remain per-record even if final topology later batches
    construction cutters. Batched backend cuts must recover provenance through
    the original per-interface, per-owner, and per-atomic-volume context records.
    """

    atomic_volumes: tuple[AtomicVolumeRecord, ...]
    interfaces: tuple[InterfacePatchRecord, ...]
    rings: tuple[RingPatchRecord, ...]
    atomic_by_id: Mapping[str, AtomicVolumeRecord]
    interface_by_id: Mapping[str, InterfacePatchRecord]
    rings_by_parent_interface_id: Mapping[str, tuple[RingPatchRecord, ...]]
    interfaces_by_kind: Mapping[InterfaceKindLiteral, tuple[InterfacePatchRecord, ...]]
    interfaces_by_owner: Mapping[str, tuple[InterfacePatchRecord, ...]]
    atomic_by_owner_semantic_id: Mapping[str, tuple[AtomicVolumeRecord, ...]]


def build_route_materialization_context(
    *,
    atomic_volumes: Iterable[AtomicVolumeRecord],
    interfaces: Iterable[InterfacePatchRecord],
    rings: Iterable[RingPatchRecord],
) -> RouteMaterializationContext:
    """Build common indexes once for Route A/B/C materialization."""
    atomic_volume_tuple = tuple(atomic_volumes)
    interface_tuple = tuple(interfaces)
    ring_tuple = tuple(rings)

    rings_by_parent: dict[str, list[RingPatchRecord]] = defaultdict(list)
    for ring in ring_tuple:
        rings_by_parent[ring.parent_interface_id].append(ring)

    interfaces_by_kind: dict[InterfaceKindLiteral, list[InterfacePatchRecord]] = {
        kind: [] for kind in DEFAULT_INTERFACE_SOLVER_USE
    }
    interfaces_by_owner: dict[str, list[InterfacePatchRecord]] = defaultdict(list)
    for interface in interface_tuple:
        interfaces_by_kind[interface.kind].append(interface)
        for owner_semantic_id in interface.owner_semantic_ids:
            interfaces_by_owner[owner_semantic_id].append(interface)

    atomic_by_owner: dict[str, list[AtomicVolumeRecord]] = defaultdict(list)
    for atomic_volume in atomic_volume_tuple:
        if atomic_volume.reference_owner_semantic_id is not None:
            atomic_by_owner[atomic_volume.reference_owner_semantic_id].append(
                atomic_volume
            )

    return RouteMaterializationContext(
        atomic_volumes=atomic_volume_tuple,
        interfaces=interface_tuple,
        rings=ring_tuple,
        atomic_by_id={volume.atomic_id: volume for volume in atomic_volume_tuple},
        interface_by_id={
            interface.interface_id: interface for interface in interface_tuple
        },
        rings_by_parent_interface_id={
            parent_id: tuple(parent_rings)
            for parent_id, parent_rings in rings_by_parent.items()
        },
        interfaces_by_kind={
            kind: tuple(kind_interfaces)
            for kind, kind_interfaces in interfaces_by_kind.items()
        },
        interfaces_by_owner={
            owner_id: tuple(owner_interfaces)
            for owner_id, owner_interfaces in interfaces_by_owner.items()
        },
        atomic_by_owner_semantic_id={
            owner_id: tuple(owner_volumes)
            for owner_id, owner_volumes in atomic_by_owner.items()
        },
    )
