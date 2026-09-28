#include "test_framework.hpp"

#include "rigid2d/world.hpp"

using namespace rigid2d;

namespace {

WorldConfig groundCfg() {
    WorldConfig cfg;
    cfg.gravity = {0.0, -9.81};
    return cfg;
}

// Builds a large static ground box with its top surface at y = topY.
// Ground friction defaults to 1.0; the pair coefficient is the geometric
// mean sqrt(f_ground * f_body), so a zero-friction ground would disable
// friction for everything resting on it.
Body* makeGround(World& w, double topY = 0.0, double friction = 1.0) {
    Body* g = w.add(
        Body::aabb({0.0, topY - 10.0}, {100.0, 10.0}, 1.0, 0.0, friction));
    g->makeStatic();
    return g;
}

}  // namespace

R2_TEST(ball_comes_to_rest_on_static_ground) {
    World world(groundCfg());
    makeGround(world, 0.0);
    Body* ball = world.add(Body::circle({0.0, 10.0}, 0.5, 1.0, 0.2, 0.6));

    for (int i = 0; i < 60 * 5; ++i) world.step();

    // Sleeps and settles with the bottom touching the ground (within slop).
    CHECK(ball->asleep);
    CHECK_NEAR_VEC2(ball->velocity, 0.0, 0.0, 1e-12);
    CHECK(ball->position.y <= 0.5 + world.config().slop + 1e-9);
    CHECK(ball->position.y >= 0.5 - world.config().slop - 0.01);
}

R2_TEST(static_ground_never_gains_velocity) {
    World world(groundCfg());
    Body* ground = makeGround(world, 0.0);
    Body* heavy = world.add(Body::circle({0.0, 5.0}, 0.5, 1000.0, 0.3, 0.5));

    for (int i = 0; i < 60 * 3; ++i) world.step();

    CHECK(ground->isStatic);
    CHECK_NEAR(ground->velocity.x, 0.0, 0.0);
    CHECK_NEAR(ground->velocity.y, 0.0, 0.0);
    CHECK_NEAR(ground->position.y, -10.0, 1e-12);
    CHECK(heavy->position.y <= 0.5 + world.config().slop + 1e-9);
}

R2_TEST(bounce_velocity_matches_restitution) {
    // Zero gravity: impact speed equals the launch speed, so the separation
    // speed ratio is exactly the restitution coefficient.
    WorldConfig cfg = groundCfg();
    cfg.gravity = {0.0, 0.0};
    cfg.allowSleep = false;
    World world(cfg);
    makeGround(world, 0.0);
    Body* ball = world.add(Body::circle({0.0, 0.3}, 0.2, 1.0, 0.9, 0.0));
    ball->velocity = {0.0, -10.0};

    double bestUp = 0.0;
    for (int i = 0; i < 60 * 2; ++i) {
        world.step();
        if (ball->velocity.y > bestUp) bestUp = ball->velocity.y;
    }
    // Separation speed = e * impact speed = 0.9 * 10 = 9 m/s.
    CHECK_NEAR(bestUp, 9.0, 0.05);
}

R2_TEST(inelastic_drop_does_not_bounce) {
    WorldConfig cfg = groundCfg();
    World world(cfg);
    makeGround(world, 0.0);
    Body* ball = world.add(Body::circle({0.0, 3.0}, 0.5, 1.0, 0.0, 0.8));

    for (int i = 0; i < 60 * 4; ++i) world.step();

    CHECK(ball->asleep);
    CHECK_NEAR(ball->velocity.y, 0.0, 1e-12);
    CHECK(ball->position.y < 0.5 + 0.02);
}

R2_TEST(stack_of_boxes_settles_without_exploding) {
    World world(groundCfg());
    makeGround(world, 0.0);
    const int n = 5;
    std::vector<Body*> stack;
    for (int i = 0; i < n; ++i) {
        Body* box =
            world.add(Body::aabb({0.0, 0.5 + i * 1.02}, {0.5, 0.5},
                                 1.0, 0.0, 0.6));
        stack.push_back(box);
    }

    for (int i = 0; i < 60 * 6; ++i) world.step();

    // The whole stack sleeps.
    bool allAsleep = true;
    for (Body* b : stack) {
        if (!b->asleep) allAsleep = false;
    }
    CHECK_MSG(allAsleep, "expected entire stack to be asleep");

    // Stays vertical (no sideways explosion) and ordered bottom-to-top.
    for (int i = 0; i < n; ++i) {
        CHECK(std::fabs(stack[i]->position.x) < 0.05);
        CHECK_NEAR(stack[i]->velocity.x, 0.0, 1e-12);
        CHECK_NEAR(stack[i]->velocity.y, 0.0, 1e-12);
        if (i > 0) {
            CHECK(stack[i]->position.y > stack[i - 1]->position.y);
        }
    }
}

R2_TEST(sleeping_stack_wakes_when_hit) {
    World world(groundCfg());
    makeGround(world, 0.0);
    Body* bottom = world.add(Body::aabb({0.0, 0.5}, {0.5, 0.5}, 1.0, 0.0, 0.6));
    Body* top = world.add(Body::aabb({0.0, 1.5}, {0.5, 0.5}, 1.0, 0.0, 0.6));
    for (int i = 0; i < 60 * 3; ++i) world.step();
    CHECK(top->asleep && bottom->asleep);

    // A fast projectile strikes the stack from the side.
    Body* bullet = world.add(Body::circle({-5.0, 0.5}, 0.25, 1.0, 0.1, 0.0));
    bullet->velocity = {15.0, 0.0};
    for (int i = 0; i < 60; ++i) world.step();

    // At least the bottom box must be awake and knocked sideways.
    CHECK(!bottom->asleep);
    CHECK(std::fabs(bottom->velocity.x) > 0.5 ||
          std::fabs(bottom->position.x) > 0.1);
}

R2_TEST(resting_stack_does_not_accumulate_energy) {
    WorldConfig cfg = groundCfg();
    cfg.allowSleep = false;  // hardest case: sleeping cannot mask jitter
    World world(cfg);
    makeGround(world, 0.0);
    // Start the boxes exactly in contact: a genuine resting stack, so any
    // kinetic energy present afterwards can only come from numerical
    // error, never from a physical fall.
    for (int i = 0; i < 4; ++i) {
        world.add(Body::aabb({0.0, 0.5 + i * 1.0}, {0.5, 0.5},
                             1.0, 0.0, 0.6));
    }

    // Let the stack settle for a second, then watch steady-state energy for
    // a long interval. It must not grow (no energy injection / perpetual
    // bounce from resting contacts).
    for (int i = 0; i < 60; ++i) world.step();
    double maxKe = 0.0;
    double firstKe = world.totalKineticEnergy();
    for (int i = 0; i < 60 * 20; ++i) {
        world.step();
        maxKe = std::max(maxKe, world.totalKineticEnergy());
    }
    CHECK(maxKe < 1e-3);
    CHECK_MSG(maxKe <= firstKe + 1e-9,
              "steady-state energy grew from " + std::to_string(firstKe) +
                  " to " + std::to_string(maxKe));
}

R2_TEST(friction_stops_sliding_body) {
    WorldConfig cfg = groundCfg();
    cfg.allowSleep = true;
    World world(cfg);
    makeGround(world, 0.0);
    Body* box = world.add(Body::aabb({0.0, 0.5}, {0.5, 0.5}, 1.0, 0.0, 0.8));
    box->velocity = {4.0, 0.0};

    for (int i = 0; i < 60 * 5; ++i) world.step();

    // Kinetic friction bleeds the horizontal velocity away; the box sleeps.
    CHECK(box->asleep);
    CHECK_NEAR(box->velocity.x, 0.0, 1e-9);
    CHECK(box->position.y < 0.5 + 0.02);
}
