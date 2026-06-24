"""Route A materialization contract."""

from semantic_geometry_builder.models import RouteMaterializationRecord
from semantic_geometry_builder.route_materialization.context import (
    RouteMaterializationContext,
)


def materialize_route_a(
    context: RouteMaterializationContext,
) -> RouteMaterializationRecord:
    """Materialize Route A mixed surface-sheet / PEC-shell geometry intent.

    Route A lowers face metals and airbridge decks attached through
    `attached_face_metal_semantic_id` to solver-active `surface_sheet`
    conductors. Indium bumps and airbridge posts are not embedded sheets; they
    use construction bodies to cut host regions and leave solver-facing PEC
    `cutout_boundary_shell` surfaces. This stage owns those host cuts,
    sheet-imprint operations, electrical-net groups, surviving MS/MA/SA
    interfaces, audit-only MM/SS/AA candidates, and rings for surviving
    solver-active interfaces.

    Route A should emit per-body cut-host intent for bump/post blockers and
    separate sheet-imprint intent for face-metal/deck sheets. `surface_sheet`
    conductors must not enter cutter groups; final topology later applies any
    compatible cutter batching while preserving per-object provenance.
    """
    del context
    raise NotImplementedError("materialize_route_a")
