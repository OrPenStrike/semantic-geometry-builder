"""Final physical-group export helpers."""

from __future__ import annotations

from semantic_geometry_builder.models import (
    BackendEntityTagRecord,
    ConstructionPlanRecord,
    FinalPhysicalGroupRecord,
    TagPlanRecord,
)


def export_physical_group_records(
    plan: ConstructionPlanRecord,
) -> tuple[FinalPhysicalGroupRecord, ...]:
    """Convert backend-tagged tag plans to final physical group records."""
    records: list[FinalPhysicalGroupRecord] = []
    backend_tags_by_source = _backend_tags_by_source(plan.backend_entity_tags)
    for tags in _group_tag_plans(plan.tags):
        first_tag = tags[0]
        entity_tags = _physical_entity_tags(tags, backend_tags_by_source)
        if not entity_tags:
            raise NotImplementedError(
                f"{first_tag.physical_name} has no backend entity tags yet"
            )
        records.append(
            FinalPhysicalGroupRecord(
                physical_name=first_tag.physical_name,
                dimension=first_tag.dimension,
                route=plan.route,
                role=first_tag.role,
                source_record_id=first_tag.physical_name,
                solver_use=first_tag.solver_use,
                entity_tags=entity_tags,
                metadata={
                    "source_record_ids": tuple(tag.source_record_id for tag in tags),
                },
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


def _group_tag_plans(
    tags: tuple[TagPlanRecord, ...],
) -> tuple[tuple[TagPlanRecord, ...], ...]:
    grouped: dict[tuple[str, int, str, str], list[TagPlanRecord]] = {}
    for tag in tags:
        grouped.setdefault(
            (tag.physical_name, tag.dimension, tag.role, tag.solver_use),
            [],
        ).append(tag)
    return tuple(tuple(items) for items in grouped.values())


def _physical_entity_tags(
    tags: tuple[TagPlanRecord, ...],
    backend_tags_by_source: dict[tuple[str, str], tuple[tuple[int, int], ...]],
) -> tuple[int, ...]:
    entity_tags: list[int] = []
    seen: set[int] = set()
    for tag in tags:
        for dimension, entity_tag in backend_tags_by_source.get(
            (tag.source_record_kind, tag.source_record_id),
            (),
        ):
            if dimension != tag.dimension or entity_tag in seen:
                continue
            entity_tags.append(entity_tag)
            seen.add(entity_tag)
    return tuple(entity_tags)

