"""Terminal-only test suite for the physics2d engine (pytest, no rendering)."""

import math

import pytest

from physics2d import Body, Vec2, World, collide

DT = 1.0 / 60.0


def run(world, seconds, dt=DT):
    steps = round(seconds / dt)
    for _ in range(steps):
        world.step(dt)
    return steps


# ---------------------------------------------------------------- free fall

def test_free_fall_matches_analytic_solution():
    w = World(gravity=Vec2(0, -10))
    b = w.add_body(Body.circle(Vec2(0, 100), radius=1, mass=2))
    t = 1.0
    run(w, t)
    # Semi-implicit Euler overshoots slightly vs the analytic solution; the
    # error must stay small and bounded.
    assert b.velocity.y == pytest.approx(-10 * t, abs=0.2)
    assert b.position.y == pytest.approx(100 - 0.5 * 10 * t * t, abs=0.2)


def test_constant_external_force():
    w = World(gravity=Vec2(0, 0))
    b = w.add_body(Body.circle(Vec2(0, 0), radius=1, mass=2))
    b.apply_force(Vec2(4, 0))
    w.step(DT)
    # v = F/m * dt ; force is cleared after the step.
    assert b.velocity.x == pytest.approx(4 / 2 * DT)
    assert b.force == Vec2(0, 0)


def test_determinism_same_inputs_same_results():
    def build():
        w = World()
        w.add_body(Body.circle(Vec2(-5, 0), 1, velocity=Vec2(3, 0), mass=1,
                               restitution=0.8))
        w.add_body(Body.circle(Vec2(5, 0), 1, velocity=Vec2(-3, 0), mass=1,
                               restitution=0.8))
        return w

    w1, w2 = build(), build()
    run(w1, 2.0)
    run(w2, 2.0)
    for b1, b2 in zip(w1.bodies, w2.bodies):
        assert b1.position == b2.position
        assert b1.velocity == b2.velocity


# ------------------------------------------------------------- collisions

def test_circle_circle_detection_normal_and_depth():
    a = Body.circle(Vec2(0, 0), 1)
    b = Body.circle(Vec2(1.5, 0), 1)
    m = collide(a, b)
    assert m is not None
    assert m.normal == Vec2(1, 0)
    assert m.penetration == pytest.approx(0.5)
    assert collide(a, Body.circle(Vec2(3, 0), 1)) is None


def test_aabb_aabb_detection_normal_and_depth():
    a = Body.aabb(Vec2(0, 0), 1, 1)
    b = Body.aabb(Vec2(0, 1.5), 1, 1)
    m = collide(a, b)
    assert m is not None
    assert m.normal == Vec2(0, 1)
    assert m.penetration == pytest.approx(0.5)


def test_circle_aabb_detection():
    c = Body.circle(Vec2(0, 1.5), 1)
    box = Body.aabb(Vec2(0, 0), 2, 1)
    m = collide(c, box)
    assert m is not None
    assert m.normal == Vec2(0, -1)  # from circle toward box
    assert m.penetration == pytest.approx(0.5)
    # Flipped order must give the flipped normal.
    m2 = collide(box, c)
    assert m2.normal == Vec2(0, 1)


def test_elastic_collision_conserves_momentum_and_energy():
    w = World(gravity=Vec2(0, 0))
    a = w.add_body(Body.circle(Vec2(-5, 0), 1, velocity=Vec2(4, 0), mass=2,
                               restitution=1.0, friction=0.0))
    b = w.add_body(Body.circle(Vec2(5, 0), 1, velocity=Vec2(-2, 0), mass=1,
                               restitution=1.0, friction=0.0))
    p0 = w.total_momentum()
    e0 = w.total_kinetic_energy()
    run(w, 3.0)
    p1 = w.total_momentum()
    e1 = w.total_kinetic_energy()
    assert p1.x == pytest.approx(p0.x, abs=1e-6)
    assert p1.y == pytest.approx(p0.y, abs=1e-6)
    assert e1 == pytest.approx(e0, rel=1e-6)
    # Analytic 1D elastic solution: v1' = 0, v2' = 6.
    assert a.velocity.x == pytest.approx(0.0, abs=1e-6)
    assert b.velocity.x == pytest.approx(6.0, abs=1e-6)


def test_perfectly_inelastic_collision_conserves_momentum():
    w = World(gravity=Vec2(0, 0))
    a = w.add_body(Body.circle(Vec2(-5, 0), 1, velocity=Vec2(3, 0), mass=1,
                               restitution=0.0, friction=0.0))
    b = w.add_body(Body.circle(Vec2(5, 0), 1, velocity=Vec2(-1, 0), mass=3,
                               restitution=0.0, friction=0.0))
    p0 = w.total_momentum()
    run(w, 3.0)
    p1 = w.total_momentum()
    assert p1.x == pytest.approx(p0.x, abs=1e-6)
    # e = 0: relative normal velocity after impact is ~0 (they move together).
    rel = (b.velocity - a.velocity).x
    assert abs(rel) < 0.1
    # Energy must not increase.
    assert w.total_kinetic_energy() <= 0.5 * 1 * 9 + 0.5 * 3 * 1 + 1e-9


def test_static_ground_stops_falling_body():
    w = World(gravity=Vec2(0, -10))
    ground = w.add_body(Body.aabb(Vec2(0, -1), 50, 1, is_static=True,
                                  friction=0.8))
    ball = w.add_body(Body.circle(Vec2(0, 5), 1, mass=1, restitution=0.0,
                                  friction=0.8))
    run(w, 3.0)
    # Resting on top of the ground: centre near y = 1 (ground top 0 + radius).
    assert ball.position.y == pytest.approx(1.0, abs=0.05)
    assert ball.velocity.length() < 0.1
    # Static body never gains velocity.
    assert ground.velocity == Vec2(0, 0)
    assert ground.position == Vec2(0, -1)


def test_elastic_bounce_off_static_ground():
    w = World(gravity=Vec2(0, -10))
    w.add_body(Body.aabb(Vec2(0, -1), 50, 1, is_static=True, restitution=1.0))
    ball = w.add_body(Body.circle(Vec2(0, 5), 1, mass=1, restitution=1.0,
                                  friction=0.0))
    run(w, 0.9)
    assert ball.velocity.y < 0  # falling before impact
    run(w, 0.5)
    assert ball.velocity.y > 0  # bounced back up


def test_stacking_is_stable_and_loses_energy():
    w = World(gravity=Vec2(0, -10))
    w.add_body(Body.aabb(Vec2(0, -0.5), 50, 0.5, is_static=True, friction=0.9))
    boxes = [
        w.add_body(Body.aabb(Vec2(0, 0.5 + i * 1.001), 0.5, 0.5, mass=1,
                             restitution=0.0, friction=0.9))
        for i in range(3)
    ]
    run(w, 4.0)
    for i, b in enumerate(boxes):
        # Each box rests near its slot: y ~= 0.5, 1.5, 2.5
        assert b.position.y == pytest.approx(0.5 + i, abs=0.1)
        assert b.velocity.length() < 0.1
    # No runaway energy: total KE of a settled stack must be near zero.
    assert w.total_kinetic_energy() < 0.01


def test_friction_slows_sliding_body():
    w = World(gravity=Vec2(0, -10))
    w.add_body(Body.aabb(Vec2(0, -0.5), 50, 0.5, is_static=True, friction=1.0))
    slider = w.add_body(Body.aabb(Vec2(0, 0.5), 0.5, 0.5, mass=1,
                                  velocity=Vec2(5, 0), restitution=0.0,
                                  friction=1.0))
    run(w, 2.0)
    assert abs(slider.velocity.x) < 0.5  # friction stopped the sliding


# ------------------------------------------------------- boundary inputs

def test_zero_and_negative_mass_are_immovable():
    w = World(gravity=Vec2(0, -10))
    z = w.add_body(Body.circle(Vec2(0, 10), 1, mass=0.0))
    n = w.add_body(Body.circle(Vec2(5, 10), 1, mass=-3.0))
    run(w, 1.0)
    assert z.position == Vec2(0, 10) and z.velocity == Vec2(0, 0)
    assert n.position == Vec2(5, 10) and n.velocity == Vec2(0, 0)


def test_extreme_mass_ratio_is_stable():
    w = World(gravity=Vec2(0, 0))
    heavy = w.add_body(Body.circle(Vec2(5, 0), 1, mass=1e6, restitution=0.5))
    light = w.add_body(Body.circle(Vec2(-5, 0), 1, velocity=Vec2(10, 0),
                                   mass=1e-6, restitution=0.5))
    run(w, 2.0)
    # The heavy body barely moves; the light one bounces off.
    assert heavy.velocity.length() < 1e-3
    assert light.velocity.x < 0
    assert math.isfinite(light.position.x)


def test_initial_overlap_is_separated():
    w = World(gravity=Vec2(0, 0))
    a = w.add_body(Body.circle(Vec2(0, 0), 1, mass=1))
    b = w.add_body(Body.circle(Vec2(0.5, 0), 1, mass=1))
    run(w, 1.0)
    dist = (b.position - a.position).length()
    assert dist == pytest.approx(2.0, abs=0.05)


def test_concentric_circles_get_deterministic_normal():
    m = collide(Body.circle(Vec2(1, 1), 1), Body.circle(Vec2(1, 1), 1))
    assert m is not None
    assert m.normal == Vec2(1, 0)
    assert m.penetration == pytest.approx(2.0)


def test_high_speed_body_is_clamped_and_finite():
    w = World(gravity=Vec2(0, 0))
    fast = w.add_body(Body.circle(Vec2(0, 0), 1, velocity=Vec2(1e6, 0),
                                  mass=1))
    w.step(DT)
    assert fast.velocity.length() <= 1000.0  # MAX_SPEED clamp
    run(w, 1.0)
    assert math.isfinite(fast.position.x)


def test_invalid_inputs_raise():
    with pytest.raises(ValueError):
        Body.circle(Vec2(0, 0), radius=0)
    with pytest.raises(ValueError):
        Body.aabb(Vec2(0, 0), 0, 1)
    with pytest.raises(ValueError):
        Body("triangle", Vec2(0, 0))
    w = World()
    with pytest.raises(ValueError):
        w.step(0.0)


def test_restitution_and_friction_are_clamped():
    b = Body.circle(Vec2(0, 0), 1, restitution=5.0, friction=-1.0)
    assert b.restitution == 1.0
    assert b.friction == 0.0
