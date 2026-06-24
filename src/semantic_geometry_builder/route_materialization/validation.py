"""Route materialization validation contract."""

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
    del record, context
    raise NotImplementedError("validate_route_materialization")
