"""Route-specific materialization API."""

from semantic_geometry_builder.route_materialization.context import (
    RouteMaterializationContext,
    build_route_materialization_context,
)
from semantic_geometry_builder.route_materialization.route_a import materialize_route_a
from semantic_geometry_builder.route_materialization.route_b import materialize_route_b
from semantic_geometry_builder.route_materialization.route_c import materialize_route_c
from semantic_geometry_builder.route_materialization.validation import (
    validate_route_materialization,
)

__all__ = [
    "RouteMaterializationContext",
    "build_route_materialization_context",
    "materialize_route_a",
    "materialize_route_b",
    "materialize_route_c",
    "validate_route_materialization",
]
