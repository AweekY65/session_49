#include "test_framework.hpp"

#include <cmath>
#include <limits>

#include "rigid2d/collision.hpp"
#include "rigid2d/world.hpp"

using namespace rigid2d;

// ---------------------------------------------------------------------------
// Manifold geometry tests (normal + penetration depth)
// ---------------------------------------------------------------------------

R2_TEST(manifold_circle_circle_separated_and_touching) {
    Body a = Body::circle({0.0, 0.0}, 1.0, 1.0);
    Body b = Body::circle({3.0, 0.0}, 1.0, 1.0);  // gap of 1
    Contact c;
    CHECK(!collide(a, b, c));

    b.position = {1.5, 0.0};  // overlap 0.5
    CHECK(collide(a, b, c));
    CHECK_NEAR(c.normal.x, 1.0, 1e-12);
    CHECK_NEAR(c.normal.y, 0.0, 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);

    // Diagonal pair: normal length must be unit and point a -> b.
    b.position = {0.0, 1.5};
    CHECK(collide(a, b, c));
    CHECK_NEAR(c.normal.x, 0.0, 1e-12);
    CHECK_NEAR(c.normal.y, 1.0, 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);
}

R2_TEST(manifold_circle_circle_coincident_centers_is_handled) {
    Body a = Body::circle({0.0, 0.0}, 1.0, 1.0);
    Body b = Body::circle({0.0, 0.0}, 1.0, 1.0);
    Contact c;
    CHECK(collide(a, b, c));
    // Finite unit normal and penetration equal to the radius sum.
    CHECK_NEAR(length(c.normal), 1.0, 1e-12);
    CHECK_NEAR(c.penetration, 2.0, 1e-12);

    // Solver must not blow up: two coincident bodies separate and stay sane.
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    cfg.allowSleep = false;
    World world(cfg);
    Body* p = world.add(Body::circle({0.0, 0.0}, 1.0, 1.0));
    Body* q = world.add(Body::circle({0.0, 0.0}, 1.0, 1.0));
    for (int i = 0; i < 120; ++i) world.step();
    CHECK(std::isfinite(p->position.x));
    CHECK(std::isfinite(q->position.x));
    CHECK(std::isfinite(p->velocity.x));
    CHECK(std::isfinite(q->velocity.x));
}

R2_TEST(manifold_aabb_aabb_axis_of_least_overlap) {
    Body a = Body::aabb({0.0, 0.0}, {1.0, 1.0}, 1.0);
    Body b = Body::aabb({1.5, 0.2}, {1.0, 1.0}, 1.0);
    Contact c;
    CHECK(collide(a, b, c));
    // x overlap 0.5 < y overlap 1.8 -> normal +x.
    CHECK_NEAR(c.normal.x, 1.0, 1e-12);
    CHECK_NEAR(c.normal.y, 0.0, 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);

    b.position = {0.2, -1.5};
    CHECK(collide(a, b, c));
    // y overlap 0.5 < x overlap 1.8 -> normal -y.
    CHECK_NEAR(c.normal.x, 0.0, 1e-12);
    CHECK_NEAR(c.normal.y, -1.0, 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);

    b.position = {5.0, 0.0};
    CHECK(!collide(a, b, c));
}

R2_TEST(manifold_circle_aabb_face_and_corner) {
    Body circle = Body::circle({3.0, 0.0}, 1.0, 1.0);
    Body box = Body::aabb({0.0, 0.0}, {1.0, 1.0}, 1.0);
    Contact c;
    // Face contact from the right: gap 1.
    CHECK(!collide(circle, box, c));

    circle.position = {1.5, 0.0};  // overlap face, 0.5 deep
    CHECK(collide(circle, box, c));
    CHECK_NEAR(c.normal.x, -1.0, 1e-12);
    CHECK_NEAR(c.normal.y, 0.0, 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);

    // Corner contact at 45 degrees: nearest box point is the corner
    // (1,1); center sits (r - pen)/sqrt(2) beyond it on both axes so
    // that the penetration is exactly 0.5.
    const double off = (1.0 - 0.5) / std::sqrt(2.0);
    circle.position = {1.0 + off, 1.0 + off};
    CHECK(collide(circle, box, c));
    CHECK_NEAR(c.normal.x, -std::sqrt(0.5), 1e-12);
    CHECK_NEAR(c.normal.y, -std::sqrt(0.5), 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);
}

R2_TEST(manifold_circle_inside_aabb_pushes_out_through_face) {
    Body circle = Body::circle({-1.2, 0.0}, 0.5, 1.0);
    Body box = Body::aabb({0.0, 0.0}, {1.0, 1.0}, 1.0);
    Contact c;
    CHECK(collide(circle, box, c));
    // Circle sits 0.2 inside the box past the left face. The nearest face
    // is the left one; the normal points circle -> box, i.e. +x (toward
    // that face), so the circle's impulse (-j * n) pushes it out to -x.
    CHECK_NEAR(c.normal.x, 1.0, 1e-12);
    CHECK_NEAR(c.normal.y, 0.0, 1e-12);
    // penetration = r - distToFace = 0.5 - 0.2 = 0.3
    CHECK_NEAR(c.penetration, 0.3, 1e-12);
}

R2_TEST(manifold_aabb_circle_flips_normal) {
    Body box = Body::aabb({0.0, 0.0}, {1.0, 1.0}, 1.0);
    Body circle = Body::circle({1.5, 0.0}, 1.0, 1.0);
    Contact c;
    CHECK(collide(box, circle, c));  // a=box, b=circle
    // Normal must point box -> circle (+x), opposite of circle -> box.
    CHECK(c.a == &box);
    CHECK(c.b == &circle);
    CHECK_NEAR(c.normal.x, 1.0, 1e-12);
    CHECK_NEAR(c.penetration, 0.5, 1e-12);
}

// ---------------------------------------------------------------------------
// Boundary input rules
// ---------------------------------------------------------------------------

R2_TEST(invalid_masses_are_rejected_static_is_the_escape_hatch) {
    CHECK_THROWS(Body::circle({0.0, 0.0}, 1.0, 0.0));
    CHECK_THROWS(Body::circle({0.0, 0.0}, 1.0, -2.0));
    CHECK_THROWS(Body::circle({0.0, 0.0}, 1.0,
                              std::numeric_limits<double>::infinity()));
    CHECK_THROWS(Body::circle({0.0, 0.0}, 1.0,
                              std::numeric_limits<double>::quiet_NaN()));

    Body s = Body::aabb({0.0, 0.0}, {1.0, 1.0}, 1.0);
    s.makeStatic();
    CHECK(s.isStatic);
    CHECK(s.inverseMass == 0.0);
    CHECK(!s.movable());

    // Degenerate shapes are rejected at construction time through setMass
    // only; radius/half extents are validated in the world's substep sizing
    // indirectly. Zero-radius bodies are rejected explicitly:
    CHECK_THROWS(Body::circle({0.0, 0.0}, 0.0, 1.0));
}

R2_TEST(restitution_and_friction_are_clamped) {
    Body b = Body::circle({0.0, 0.0}, 1.0, 1.0, 5.0, -3.0);
    CHECK_NEAR(b.restitution, 1.0, 0.0);  // > 1 clamps to 1
    CHECK_NEAR(b.friction, 0.0, 0.0);     // < 0 clamps to 0

    b.setRestitution(-0.4);
    CHECK_NEAR(b.restitution, 0.0, 0.0);
    b.setFriction(1e300);
    CHECK(std::isfinite(b.friction));
}

R2_TEST(extreme_mass_ratio_behaves_like_a_wall) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    cfg.allowSleep = false;
    World world(cfg);
    const double M = 1e10;
    Body* heavy = world.add(Body::circle({0.0, 0.0}, 1.0, M, 1.0));
    Body* light = world.add(Body::circle({-2.0, 0.0}, 1.0, 1.0, 1.0));
    light->velocity = {3.0, 0.0};
    const Vec2 p0 = world.totalMomentum();
    for (int i = 0; i < 240; ++i) world.step();

    // Light body reflects almost exactly off a stationary wall:
    // v_light' ~= 3 - 2*(3)/(1+1/M) reversed => -3.
    CHECK_NEAR(light->velocity.x, -3.0, 1e-8);
    // Heavy body barely moves.
    CHECK(std::fabs(heavy->velocity.x) < 1e-7);
    // Momentum still conserved.
    CHECK_NEAR(world.totalMomentum().x, p0.x, 1e-6);
}

R2_TEST(initial_overlap_is_resolved) {
    WorldConfig cfg;
    cfg.gravity = {0.0, -2.0};
    cfg.allowSleep = false;
    World world(cfg);
    // Spawn boxes overlapping by 0.4.
    Body* a = world.add(Body::aabb({0.0, 0.0}, {1.0, 1.0}, 1.0, 0.0, 0.0));
    Body* b = world.add(Body::aabb({1.6, 0.0}, {1.0, 1.0}, 1.0, 0.0, 0.0));
    for (int i = 0; i < 300; ++i) world.step();

    // Positional correction must have pushed them apart.
    const double gap =
        std::fabs(b->position.x - a->position.x) - (a->half.x + b->half.x);
    CHECK(gap >= -world.config().slop - 1e-9);
    CHECK(std::isfinite(a->position.x));
    CHECK(std::isfinite(b->position.x));
}

R2_TEST(high_speed_body_does_not_tunnel) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    World world(cfg);
    Body* wall = world.add(Body::aabb({5.0, 0.0}, {0.5, 50.0}, 1.0));
    wall->makeStatic();
    Body* bullet = world.add(Body::circle({0.0, 0.0}, 0.25, 1.0, 0.2));
    // 100 m/s: naive one-step integration covers 1.67 m and would jump the
    // wall entirely; adaptive substepping prevents this.
    bullet->velocity = {100.0, 0.0};

    bool everRightOfWall = false;
    double maxPenetration = 0.0;
    for (int i = 0; i < 60; ++i) {
        world.step();
        if (bullet->position.x > 5.5) everRightOfWall = true;
        for (const Contact& c : world.contacts()) {
            maxPenetration = std::max(maxPenetration, c.penetration);
        }
    }
    CHECK(!everRightOfWall);
    // Bounced back to the left.
    CHECK(bullet->position.x < 4.0);
    CHECK(bullet->velocity.x < 0.0);
    CHECK(maxPenetration < 0.1);
}

R2_TEST(force_on_static_body_is_ignored) {
    World world;
    Body* g = world.add(Body::aabb({0.0, -1.0}, {10.0, 1.0}, 1.0));
    g->makeStatic();
    world.applyForce(g, {1e9, 1e9});
    world.applyImpulse(g, {1e9, 1e9});
    world.step();
    CHECK_NEAR(g->position.x, 0.0, 0.0);
    CHECK_NEAR(g->position.y, -1.0, 0.0);
    CHECK_NEAR(g->velocity.x, 0.0, 0.0);
    CHECK_NEAR(g->velocity.y, 0.0, 0.0);
}
