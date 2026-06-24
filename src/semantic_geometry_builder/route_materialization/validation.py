"""Route materialization validation contract."""

from collections import Counter

from semantic_geometry_builder.models import RouteMaterializationRecord
from semantic_geometry_builder.route_materialization.context import (
    RouteMaterializationContext,
)


def validate_route_materialization(
    record: RouteMaterializationRecord,
    context: RouteMaterializationContext,
) -> RouteMaterializationRecord:
    """Validate common invariants after route-specific materialization.

    This should check route consistency, existing ids, non-overlapping parent
    and ring physical intent, airbridge deck route inheritance through
    `attached_face_metal_semantic_id`, and whether route-specific operations
    reference live context records.
    """
    errors: list[str] = []

    if record.route not in {"A", "B", "C"}:
        errors.append(f"unknown route: {record.route!r}")

    for label, ids in (
        ("atomic_volumes", [volume.atomic_id for volume in context.atomic_volumes]),
        ("interfaces", [interface.interface_id for interface in context.interfaces]),
        ("rings", [ring.ring_id for ring in context.rings]),
    ):
        duplicate_ids = sorted(
            item_id for item_id, count in Counter(ids).items() if count > 1
        )
        if duplicate_ids:
            errors.append(f"{label} contains duplicate ids: {duplicate_ids!r}")

    atomic_ids = set(context.atomic_by_id)
    interface_ids = set(context.interface_by_id)
    ring_by_id = {ring.ring_id: ring for ring in context.rings}

    for label, ids, known_ids in (
        ("surviving_volume_ids", record.surviving_volume_ids, atomic_ids),
        ("removed_volume_ids", record.removed_volume_ids, atomic_ids),
        ("reassigned_volume_owners", record.reassigned_volume_owners, atomic_ids),
        ("surviving_interface_ids", record.surviving_interface_ids, interface_ids),
        (
            "pec_boundary_interface_ids",
            record.pec_boundary_interface_ids,
            interface_ids,
        ),
        ("surviving_ring_ids", record.surviving_ring_ids, set(ring_by_id)),
    ):
        unknown_ids = set(ids) - known_ids
        if unknown_ids:
            errors.append(f"{label} contains unknown ids: {sorted(unknown_ids)!r}")

    construction_ids = set(record.construction_body_ids)
    cut_reference_ids = atomic_ids | construction_ids
    listed_sheet_ids = set(record.surviving_interface_ids) | set(
        record.pec_boundary_interface_ids
    )
    for group_members in (
        *record.electrical_net_groups.values(),
        *record.boundary_shell_groups.values(),
    ):
        listed_sheet_ids.update(group_members)
    sheet_reference_ids = listed_sheet_ids or (atomic_ids | interface_ids)

    for operation in record.cut_host_operations:
        if operation.construction_body_id not in cut_reference_ids:
            errors.append(
                f"{operation.operation_id} references unknown construction body "
                f"{operation.construction_body_id!r}"
            )
        if operation.host_solution_volume_id not in cut_reference_ids:
            errors.append(
                f"{operation.operation_id} references unknown host solution volume "
                f"{operation.host_solution_volume_id!r}"
            )

    for operation in record.sheet_imprint_operations:
        if operation.construction_body_id not in cut_reference_ids:
            errors.append(
                f"{operation.operation_id} references unknown construction body "
                f"{operation.construction_body_id!r}"
            )
        if operation.sheet_entity_id not in sheet_reference_ids:
            errors.append(
                f"{operation.operation_id} references unknown sheet entity "
                f"{operation.sheet_entity_id!r}"
            )

    parent_interfaces_with_live_rings = {
        ring_by_id[ring_id].parent_interface_id
        for ring_id in record.surviving_ring_ids
        if ring_id in ring_by_id
    }
    overlapping_parent_ids = parent_interfaces_with_live_rings & (
        set(record.surviving_interface_ids) | set(record.pec_boundary_interface_ids)
    )
    if overlapping_parent_ids:
        errors.append(
            "live ring children overlap live parent interfaces: "
            f"{sorted(overlapping_parent_ids)!r}"
        )

    if errors:
        raise ValueError("; ".join(errors))
    return record
