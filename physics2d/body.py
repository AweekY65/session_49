"""Rigid body definitions: circles and axis-aligned boxes (AABB)."""

from __future__ import annotations

from .vec2 import Vec2

CIRCLE = "circle"
AABB = "aabb"

# Bodies whose speed exceeds this are clamped each step. This bounds how far a
# body can tunnel in one fixed step (max_speed * dt) and keeps the simulation
# stable for extreme inputs. Documented behaviour, not CCD.
MAX_SPEED = 1000.0


class Body:
    """A 2D rigid body (circle or AABB).

    Attributes:
        shape: CIRCLE or AABB.
        position: center of the body (Vec2).
        velocity: linear velocity (Vec2).
        mass: mass in arbitrary units. mass <= 0 is treated as infinite mass
            (inv_mass == 0), i.e. the body is immovable.
        restitution: bounciness in [0, 1]. 0 = perfectly inelastic.
        friction: Coulomb friction coefficient (>= 0).
        is_static: if True the body never moves and has infinite mass.
        radius: circle radius (CIRCLE only).
        half_width, half_height: box half extents (AABB only).
    """

    _next_id = 0

    def __init__(
        self,
        shape: str,
        position: Vec2,
        velocity: Vec2 | None = None,
        mass: float = 1.0,
        restitution: float = 0.0,
        friction: float = 0.5,
        is_static: bool = False,
        radius: float = 0.0,
        half_width: float = 0.0,
        half_height: float = 0.0,
    ):
        if shape not in (CIRCLE, AABB):
            raise ValueError(f"unknown shape: {shape!r}")
        if shape == CIRCLE and radius <= 0.0:
            raise ValueError("circle radius must be > 0")
        if shape == AABB and (half_width <= 0.0 or half_height <= 0.0):
            raise ValueError("AABB half extents must be > 0")

        self.id = Body._next_id
        Body._next_id += 1

        self.shape = shape
        self.position = position.copy()
        self.velocity = velocity.copy() if velocity is not None else Vec2()
        self.force = Vec2()  # accumulated this step, cleared after integrate

        self.is_static = bool(is_static)
        if self.is_static or mass <= 0.0:
            # Zero / negative mass is defined as infinite mass (immovable).
            self.mass = 0.0
            self.inv_mass = 0.0
        else:
            self.mass = float(mass)
            self.inv_mass = 1.0 / self.mass

        self.restitution = min(max(float(restitution), 0.0), 1.0)
        self.friction = max(float(friction), 0.0)

        self.radius = float(radius)
        self.half_width = float(half_width)
        self.half_height = float(half_height)

    @classmethod
    def circle(cls, position, radius, **kw) -> "Body":
        return cls(CIRCLE, position, radius=radius, **kw)

    @classmethod
    def aabb(cls, position, half_width, half_height, **kw) -> "Body":
        return cls(
            AABB, position, half_width=half_width, half_height=half_height, **kw
        )

    @property
    def is_immovable(self) -> bool:
        return self.inv_mass == 0.0

    def apply_force(self, f: Vec2) -> None:
        self.force = self.force + f

    def aabb_bounds(self):
        """Return (min_x, min_y, max_x, max_y) of the body's bounding box."""
        if self.shape == CIRCLE:
            r = self.radius
            return (
                self.position.x - r,
                self.position.y - r,
                self.position.x + r,
                self.position.y + r,
            )
        return (
            self.position.x - self.half_width,
            self.position.y - self.half_height,
            self.position.x + self.half_width,
            self.position.y + self.half_height,
        )
