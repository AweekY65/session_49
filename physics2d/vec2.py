"""2D vector math (pure stdlib, deterministic)."""

from __future__ import annotations

import math


class Vec2:
    __slots__ = ("x", "y")

    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __neg__(self) -> "Vec2":
        return Vec2(-self.x, -self.y)

    def __mul__(self, k: float) -> "Vec2":
        return Vec2(self.x * k, self.y * k)

    __rmul__ = __mul__

    def __truediv__(self, k: float) -> "Vec2":
        return Vec2(self.x / k, self.y / k)

    def __eq__(self, other) -> bool:
        return (
            isinstance(other, Vec2)
            and self.x == other.x
            and self.y == other.y
        )

    def __repr__(self) -> str:
        return f"Vec2({self.x!r}, {self.y!r})"

    def copy(self) -> "Vec2":
        return Vec2(self.x, self.y)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: "Vec2") -> float:
        """Scalar z-component of the 2D cross product."""
        return self.x * other.y - self.y * other.x

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def length_sq(self) -> float:
        return self.x * self.x + self.y * self.y

    def normalized(self) -> "Vec2":
        n = self.length()
        if n == 0.0:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / n, self.y / n)

    def perp(self) -> "Vec2":
        """90-degree counter-clockwise rotation."""
        return Vec2(-self.y, self.x)
