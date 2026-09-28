"""Self-contained 2D rigid-body physics engine (no external dependencies)."""

from .body import AABB, CIRCLE, Body
from .collision import Manifold, collide
from .vec2 import Vec2
from .world import World

__all__ = [
    "AABB",
    "CIRCLE",
    "Body",
    "Manifold",
    "Vec2",
    "World",
    "collide",
]
