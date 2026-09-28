"""Narrow-phase collision detection: circle-circle, AABB-AABB, circle-AABB.

Each test returns a Manifold or None. The manifold normal always points
from body A toward body B; penetration is the (non-negative) overlap depth
along that normal.
"""

from __future__ import annotations

from .body import AABB, CIRCLE, Body
from .vec2 import Vec2


class Manifold:
    __slots__ = ("a", "b", "normal", "penetration", "contact")

    def __init__(self, a: Body, b: Body, normal: Vec2, penetration: float,
                 contact: Vec2):
        self.a = a
        self.b = b
        self.normal = normal          # unit vector, A -> B
        self.penetration = penetration
        self.contact = contact        # approximate contact point (world)

    def __repr__(self) -> str:
        return (
            f"Manifold(a={self.a.id}, b={self.b.id}, normal={self.normal!r}, "
            f"penetration={self.penetration!r})"
        )


def collide(a: Body, b: Body) -> Manifold | None:
    """Dispatch to the correct narrow-phase test for the shape pair."""
    if a.shape == CIRCLE and b.shape == CIRCLE:
        return _circle_circle(a, b)
    if a.shape == AABB and b.shape == AABB:
        return _aabb_aabb(a, b)
    if a.shape == CIRCLE and b.shape == AABB:
        return _circle_aabb(a, b)
    # AABB vs circle: flip, then flip the normal back so it still points A->B.
    m = _circle_aabb(b, a)
    if m is None:
        return None
    return Manifold(a, b, -m.normal, m.penetration, m.contact)


def _circle_circle(a: Body, b: Body) -> Manifold | None:
    d = b.position - a.position
    r = a.radius + b.radius
    dist_sq = d.length_sq()
    if dist_sq >= r * r:
        return None
    dist = dist_sq ** 0.5
    if dist > 0.0:
        normal = d / dist
        penetration = r - dist
    else:
        # Concentric: fully overlapping centres. Pick a deterministic normal.
        normal = Vec2(1.0, 0.0)
        penetration = r
    contact = a.position + normal * (a.radius - penetration * 0.5)
    return Manifold(a, b, normal, penetration, contact)


def _aabb_aabb(a: Body, b: Body) -> Manifold | None:
    d = b.position - a.position
    overlap_x = (a.half_width + b.half_width) - abs(d.x)
    if overlap_x <= 0.0:
        return None
    overlap_y = (a.half_height + b.half_height) - abs(d.y)
    if overlap_y <= 0.0:
        return None
    # Resolve along the axis of least penetration.
    if overlap_x < overlap_y:
        normal = Vec2(1.0 if d.x >= 0.0 else -1.0, 0.0)
        penetration = overlap_x
    else:
        normal = Vec2(0.0, 1.0 if d.y >= 0.0 else -1.0)
        penetration = overlap_y
    contact = Vec2(
        max(a.position.x - a.half_width, b.position.x - b.half_width)
        + min(a.position.x + a.half_width, b.position.x + b.half_width),
        max(a.position.y - a.half_height, b.position.y - b.half_height)
        + min(a.position.y + a.half_height, b.position.y + b.half_height),
    ) * 0.5
    return Manifold(a, b, normal, penetration, contact)


def _circle_aabb(circle: Body, box: Body) -> Manifold | None:
    # Closest point on the box to the circle centre.
    cx = min(max(circle.position.x,
                 box.position.x - box.half_width),
             box.position.x + box.half_width)
    cy = min(max(circle.position.y,
                 box.position.y - box.half_height),
             box.position.y + box.half_height)
    closest = Vec2(cx, cy)
    d = closest - circle.position
    dist_sq = d.length_sq()
    r = circle.radius

    if dist_sq > r * r:
        return None

    if dist_sq > 0.0:
        # Circle centre outside the box: normal from circle toward box.
        dist = dist_sq ** 0.5
        normal = d / dist
        penetration = r - dist
        contact = closest
    else:
        # Circle centre inside the box: push out along the nearest face.
        dx_min = circle.position.x - (box.position.x - box.half_width)
        dx_max = (box.position.x + box.half_width) - circle.position.x
        dy_min = circle.position.y - (box.position.y - box.half_height)
        dy_max = (box.position.y + box.half_height) - circle.position.y
        m = min(dx_min, dx_max, dy_min, dy_max)
        if m == dx_min:
            normal = Vec2(-1.0, 0.0)
        elif m == dx_max:
            normal = Vec2(1.0, 0.0)
        elif m == dy_min:
            normal = Vec2(0.0, -1.0)
        else:
            normal = Vec2(0.0, 1.0)
        penetration = r + m
        contact = circle.position + normal * (r - penetration * 0.5)
    return Manifold(circle, box, normal, penetration, contact)
