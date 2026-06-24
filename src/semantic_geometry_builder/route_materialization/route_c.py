"""Route C materialization contract."""

from semantic_geometry_builder.models import RouteMaterializationRecord
from semantic_geometry_builder.route_materialization.context import (
    RouteMaterializationContext,
)


def materialize_route_c(
    context: RouteMaterializationContext,
) -> RouteMaterializationRecord:
    """Materialize Route C retained material-volume geometry intent.

    Route C keeps conductors as `material_volume` regions. It owns
    material/contact split preservation, contact-partition rather than CAD-fuse
    intent for dissimilar conductor contacts, final MM/SS/AA audit/contact
    topology candidates, solver/postprocessing MS/MA/SA surfaces, and
    applicable rings.

    Route C must not use Route A/B temporary cutter union as a substitute for
    material topology. Dissimilar contacts, such as Al/Nb ground plane to Indium
    bump, must preserve material and semantic splits through contact
    partition/fragment logic.
    """
    del context
    raise NotImplementedError("materialize_route_c")
