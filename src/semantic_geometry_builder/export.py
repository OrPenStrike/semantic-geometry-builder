"""Final physical-group export helpers."""

from __future__ import annotations

from semantic_geometry_builder.models import (
    BackendEntityTagRecord,
    ConstructionPlanRecord,
    FinalPhysicalGroupRecord,
)


def export_physical_group_records(
    plan: ConstructionPlanRecord,
) -> tuple[FinalPhysicalGroupRecord, ...]:
    """Convert backend-tagged tag plans to final physical group records."""
    records: list[FinalPhysicalGroupRecord] = []
    backend_tags_by_source = _backend_tags_by_source(plan.backend_entity_tags)
    for tag in plan.tags:
        dim_tags = backend_tags_by_source.get(
            (tag.source_record_kind, tag.source_record_id),
            (),
        )
        entity_tags = tuple(dim_tag[1] for dim_tag in dim_tags)
        if not entity_tags:
            raise NotImplementedError(
                f"{tag.physical_name} has no backend entity tags yet"
            )
        records.append(
            FinalPhysicalGroupRecord(
                physical_name=tag.physical_name,
                dimension=tag.dimension,
                route=plan.route,
                role=tag.role,
                source_record_id=tag.source_record_id,
                solver_use=tag.solver_use,
                entity_tags=entity_tags,
                metadata=tag.metadata,
            )
        )
    return tuple(records)


def _backend_tags_by_source(
    backend_tags: tuple[BackendEntityTagRecord, ...],
) -> dict[tuple[str, str], tuple[tuple[int, int], ...]]:
    result: dict[tuple[str, str], list[tuple[int, int]]] = {}
    for record in backend_tags:
        result.setdefault(
            (record.source_record_kind, record.source_record_id),
            [],
        ).append(record.dim_tag)
    return {
        source: tuple(dim_tags)
        for source, dim_tags in result.items()
    }


