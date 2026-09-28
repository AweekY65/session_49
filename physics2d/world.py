"""Fixed-timestep 2D physics world with impulse-based collision response.

Determinism: bodies are stored and iterated in insertion order, pairs are
generated in a fixed order, no randomness and no wall-clock dependence.
Same initial state + same step count => bit-identical results.
"""

from __future__ import annotations

from .body import MAX_SPEED, Body
from .collision import Manifold, collide
from .vec2 import Vec2

# Positional correction (Baumgarte-style split): only penetration deeper than
# SLOP is corrected, and only by PERCENT per step. This lets resting contacts
# settle instead of oscillating.
PENETRATION_CORRECTION_PERCENT = 0.8
PENETRATION_SLOP = 0.01

# If the closing speed along the normal is below this, restitution is treated
# as zero. Without it, resting stacks keep micro-bouncing and never lose
# their residual energy.
RESTITUTION_VELOCITY_THRESHOLD = 1.0

# Impulse solver iterations per step (more = stiffer stacks).
SOLVER_ITERATIONS = 10


class World:
    def __init__(self, gravity: Vec2 | None = None):
        self.gravity = gravity if gravity is not None else Vec2(0.0, -9.81)
        self.bodies: list[Body] = []
        self.time = 0.0
        self.step_count = 0
        self.last_manifolds: list[Manifold] = []

    def add_body(self, body: Body) -> Body:
        self.bodies.append(body)
        return body

    # -- integration ------------------------------------------------------

    def _integrate_forces(self, dt: float) -> None:
        for b in self.bodies:
            if b.is_immovable:
                b.velocity = Vec2(0.0, 0.0)
                continue
            # Semi-implicit Euler: velocity first, position later.
            accel = self.gravity + b.force * b.inv_mass
            b.velocity = b.velocity + accel * dt
            speed = b.velocity.length()
            if speed > MAX_SPEED:
                b.velocity = b.velocity * (MAX_SPEED / speed)

    def _integrate_positions(self, dt: float) -> None:
        for b in self.bodies:
            if b.is_immovable:
                continue
            b.position = b.position + b.velocity * dt

    # -- collision response ------------------------------------------------

    def _find_manifolds(self) -> list[Manifold]:
        manifolds = []
        n = len(self.bodies)
        for i in range(n):
            for j in range(i + 1, n):
                a, b = self.bodies[i], self.bodies[j]
                if a.is_immovable and b.is_immovable:
                    continue
                m = collide(a, b)
                if m is not None:
                    manifolds.append(m)
        return manifolds

    def _resolve(self, m: Manifold) -> None:
        a, b, n = m.a, m.b, m.normal

        rv = b.velocity - a.velocity
        vel_along_normal = rv.dot(n)
        if vel_along_normal > 0.0:
            return  # separating already

        # Restitution: combine by max; kill bounce below threshold so that
        # resting stacks lose residual energy instead of jittering.
        e = max(a.restitution, b.restitution)
        if abs(vel_along_normal) < RESTITUTION_VELOCITY_THRESHOLD:
            e = 0.0

        inv_mass_sum = a.inv_mass + b.inv_mass
        if inv_mass_sum == 0.0:
            return

        # Normal impulse: j = -(1+e) * v_n / (1/m_a + 1/m_b)
        j = -(1.0 + e) * vel_along_normal / inv_mass_sum
        impulse = n * j
        a.velocity = a.velocity - impulse * a.inv_mass
        b.velocity = b.velocity + impulse * b.inv_mass

        # Friction impulse (Coulomb), tangent re-computed after normal impulse.
        rv = b.velocity - a.velocity
        t = rv - n * rv.dot(n)
        t_len = t.length()
        if t_len > 1e-12:
            t = t / t_len
            jt = -rv.dot(t) / inv_mass_sum
            mu = (a.friction * b.friction) ** 0.5
            max_friction = j * mu
            if abs(jt) > max_friction:
                jt = max_friction if jt > 0.0 else -max_friction
            friction_impulse = t * jt
            a.velocity = a.velocity - friction_impulse * a.inv_mass
            b.velocity = b.velocity + friction_impulse * b.inv_mass

    def _positional_correction(self, m: Manifold) -> None:
        a, b = m.a, m.b
        inv_mass_sum = a.inv_mass + b.inv_mass
        if inv_mass_sum == 0.0:
            return
        depth = max(m.penetration - PENETRATION_SLOP, 0.0)
        correction = m.normal * (
            PENETRATION_CORRECTION_PERCENT * depth / inv_mass_sum
        )
        a.position = a.position - correction * a.inv_mass
        b.position = b.position + correction * b.inv_mass

    # -- main entry ---------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance the simulation by exactly one fixed step `dt`."""
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        self._integrate_forces(dt)

        manifolds = self._find_manifolds()
        for _ in range(SOLVER_ITERATIONS):
            for m in manifolds:
                self._resolve(m)

        self._integrate_positions(dt)

        for m in manifolds:
            self._positional_correction(m)

        for b in self.bodies:
            b.force = Vec2(0.0, 0.0)

        self.last_manifolds = manifolds
        self.time += dt
        self.step_count += 1

    # -- diagnostics ---------------------------------------------------------

    def total_momentum(self) -> Vec2:
        p = Vec2()
        for b in self.bodies:
            if not b.is_immovable:
                p = p + b.velocity * b.mass
        return p

    def total_kinetic_energy(self) -> float:
        ke = 0.0
        for b in self.bodies:
            if not b.is_immovable:
                ke += 0.5 * b.mass * b.velocity.length_sq()
        return ke
