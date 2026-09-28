#include "test_framework.hpp"

#include "rigid2d/body.hpp"
#include "rigid2d/world.hpp"

using namespace rigid2d;

R2_TEST(free_fall_matches_semi_implicit_euler) {
    WorldConfig cfg;
    cfg.gravity = {0.0, -9.81};
    cfg.fixedDt = 1.0 / 60.0;
    cfg.allowSleep = false;
    cfg.adaptiveSubsteps = false;
    World world(cfg);
    Body* ball = world.add(Body::circle({0.0, 100.0}, 0.5, 1.0));

    const int n = 60;  // one simulated second
    for (int i = 0; i < n; ++i) world.step();

    // v(t) = g * t
    CHECK_NEAR(ball->velocity.y, -9.81, 1e-9);
    // Semi-implicit Euler: y = y0 + g*h^2 * sum_{k=1..n} k
    const double h = 1.0 / 60.0;
    const double expectedY =
        100.0 + cfg.gravity.y * h * h * (n * (n + 1) / 2.0);
    CHECK_NEAR(ball->position.y, expectedY, 1e-8);
    CHECK_NEAR(ball->velocity.x, 0.0, 1e-12);
}

R2_TEST(external_force_accelerates_f_over_m) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    cfg.adaptiveSubsteps = false;
    World world(cfg);
    Body* box = world.add(Body::aabb({0.0, 0.0}, {1.0, 1.0}, 2.0));

    // Apply F=(4,0) every step for 60 steps -> a = F/m = 2 -> v = 2.
    for (int i = 0; i < 60; ++i) {
        world.applyForce(box, {4.0, 0.0});
        world.step();
    }
    CHECK_NEAR(box->velocity.x, 2.0, 1e-9);

    // Forces are single-step requests: after a step with no applyForce the
    // body coasts at constant velocity.
    world.step();
    CHECK_NEAR(box->velocity.x, 2.0, 1e-12);
}

R2_TEST(impulse_changes_velocity_by_j_over_m) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    World world(cfg);
    Body* b = world.add(Body::circle({0.0, 0.0}, 1.0, 4.0));
    world.applyImpulse(b, {8.0, 0.0});
    // delta v = J/m = 8/4 = 2
    world.step();
    CHECK_NEAR(b->velocity.x, 2.0, 1e-12);
}

R2_TEST(determinism_identical_inputs_identical_trajectories) {
    auto build = [](int steps) {
        WorldConfig cfg;
        World w(cfg);
        Body* b1 = w.add(Body::circle({-2.0, 5.0}, 0.5, 1.0, 0.6, 0.3));
        Body* b2 = w.add(Body::circle({2.1, 6.0}, 0.5, 2.0, 0.4, 0.2));
        Body* ground = w.add(Body::aabb({0.0, -1.0}, {20.0, 1.0}, 1.0));
        ground->makeStatic();
        Body* wall = w.add(Body::aabb({8.0, 5.0}, {0.5, 5.0}, 1.0));
        wall->makeStatic();
        w.applyImpulse(b1, {3.0, 1.0});
        w.applyImpulse(b2, {-2.0, 0.5});
        for (int i = 0; i < steps; ++i) {
            w.applyForce(b1, {0.1 * std::sin(0.1 * i), 0.0});
            w.step();
        }
        return w;
    };

    World a = build(180);
    World b = build(180);
    CHECK(a.bodies().size() == b.bodies().size());
    for (std::size_t i = 0; i < a.bodies().size(); ++i) {
        const Body& ba = *a.bodies()[i];
        const Body& bb = *b.bodies()[i];
        CHECK_MSG(ba.position.x == bb.position.x,
                  "position.x differs on body " + std::to_string(i));
        CHECK_MSG(ba.position.y == bb.position.y,
                  "position.y differs on body " + std::to_string(i));
        CHECK_MSG(ba.velocity.x == bb.velocity.x,
                  "velocity.x differs on body " + std::to_string(i));
        CHECK_MSG(ba.velocity.y == bb.velocity.y,
                  "velocity.y differs on body " + std::to_string(i));
        CHECK(ba.asleep == bb.asleep);
    }
}

R2_TEST(gravity_skips_static_bodies) {
    WorldConfig cfg;
    World world(cfg);
    Body* ground = world.add(Body::aabb({0.0, 0.0}, {10.0, 1.0}, 1.0));
    ground->makeStatic();
    for (int i = 0; i < 60; ++i) world.step();
    CHECK_NEAR(ground->position.x, 0.0, 0.0);
    CHECK_NEAR(ground->position.y, 0.0, 0.0);
    CHECK_NEAR(ground->velocity.x, 0.0, 0.0);
    CHECK_NEAR(ground->velocity.y, 0.0, 0.0);
}
