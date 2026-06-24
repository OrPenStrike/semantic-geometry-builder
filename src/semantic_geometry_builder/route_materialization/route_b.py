"""Route B materialization contract."""

from semantic_geometry_builder.models import RouteMaterializationRecord
from semantic_geometry_builder.route_materialization.context import (
    RouteMaterializationContext,
)


def materialize_route_b(
    context: RouteMaterializationContext,
) -> RouteMaterializationRecord:
    """Materialize Route B cut-out PEC boundary-shell geometry intent.

    Route B lowers every conductor to construction-body cuts plus exposed PEC
    `cutout_boundary_shell` surfaces. It owns removal of final conductor
    volumes, exposed MA shell boundary selection, boundary-shell groups, PEC
    boundary interface ids, audit-only MM/SS/AA candidates, and rings for
    surviving solver-active interfaces.

    Route B should emit per-body `CutHostOperationRecord`s for semantic
    traceability even though final topology may batch compatible cutters before
    backend cuts. Exposed shell provenance must remain recoverable after
    batching.
    """
    del context
    raise NotImplementedError("materialize_route_b")
