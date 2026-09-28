#include "test_framework.hpp"

#include "rigid2d/world.hpp"

using namespace rigid2d;

namespace {

struct CollisionScene {
    std::unique_ptr<World> world;
    Body* a = nullptr;
    Body* b = nullptr;
};

// Runs two circles head-on with no gravity until their collision has fully
// resolved, and returns the world for momentum/energy checks.
CollisionScene runCircleCollision(
    double mA, double mB, const Vec2& velA, const Vec2& velB, double e,
    const Vec2& posA = {-1.0, 0.0}, const Vec2& posB = {1.0, 0.0},
    int steps = 240, double r = 0.5) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    cfg.fixedDt = 1.0 / 60.0;
    cfg.allowSleep = false;
    auto scene = CollisionScene{};
    scene.world = std::make_unique<World>(cfg);
    scene.a = scene.world->add(Body::circle(posA, r, mA, e, 0.0));
    scene.b = scene.world->add(Body::circle(posB, r, mB, e, 0.0));
    scene.a->velocity = velA;
    scene.b->velocity = velB;
    for (int i = 0; i < steps; ++i) scene.world->step();
    return scene;
}

}  // namespace

R2_TEST(elastic_circle_collision_equal_mass_swaps_velocities) {
    auto s = runCircleCollision(1.0, 1.0, {1.0, 0.0}, {-1.0, 0.0}, 1.0);

    // 1D elastic, equal masses: velocities exchange.
    CHECK_NEAR(s.a->velocity.x, -1.0, 1e-9);
    CHECK_NEAR(s.b->velocity.x, 1.0, 1e-9);
    CHECK_NEAR(s.a->velocity.y, 0.0, 1e-12);

    // Momentum (0) and kinetic energy (1 J) conserved.
    CHECK_NEAR(s.world->totalMomentum().x, 0.0, 1e-10);
    CHECK_NEAR(s.world->totalKineticEnergy(), 1.0, 1e-9);

    // Bodies separate and do not remain overlapped.
    CHECK(s.a->position.x < s.b->position.x);
    CHECK(s.b->position.x - s.a->position.x >= 1.0 - 1e-6);
}

R2_TEST(elastic_collision_unequal_mass_matches_formula) {
    auto s =
        runCircleCollision(3.0, 1.0, {2.0, 0.0}, {0.0, 0.0}, 1.0);

    // 1D elastic formulas:
    //   vA' = (mA-mB)/(mA+mB) * vA = 2/4 * 2 = 1.0
    //   vB' = 2mA/(mA+mB) * vA     = 6/4 * 2 = 3.0
    CHECK_NEAR(s.a->velocity.x, 1.0, 1e-9);
    CHECK_NEAR(s.b->velocity.x, 3.0, 1e-9);

    CHECK_NEAR(s.world->totalMomentum().x, 6.0, 1e-10);
    const double ke0 = 0.5 * 3.0 * 4.0;
    CHECK_NEAR(s.world->totalKineticEnergy(), ke0, 1e-9);
}

R2_TEST(perfectly_inelastic_equal_mass_stops_relative_motion) {
    auto s = runCircleCollision(1.0, 1.0, {1.0, 0.0}, {-1.0, 0.0}, 0.0);

    // Equal masses, e=0: both end at the center-of-mass velocity (0).
    CHECK_NEAR(s.a->velocity.x, 0.0, 1e-9);
    CHECK_NEAR(s.b->velocity.x, 0.0, 1e-9);
    CHECK_NEAR(s.world->totalMomentum().x, 0.0, 1e-12);
    CHECK_NEAR(s.world->totalKineticEnergy(), 0.0, 1e-9);
}

R2_TEST(inelastic_unequal_mass_keeps_momentum_dissipates_energy) {
    auto s =
        runCircleCollision(3.0, 1.0, {2.0, 0.0}, {0.0, 0.0}, 0.0);

    // Perfectly inelastic: common velocity = P/M = 6/4 = 1.5.
    CHECK_NEAR(s.a->velocity.x, 1.5, 1e-9);
    CHECK_NEAR(s.b->velocity.x, 1.5, 1e-9);
    CHECK_NEAR(s.world->totalMomentum().x, 6.0, 1e-10);
    // KE drops from 6 J to 0.5 * 4 * 1.5^2 = 4.5 J, never rises.
    CHECK(s.world->totalKineticEnergy() <= 4.5 + 1e-9);
    CHECK_NEAR(s.world->totalKineticEnergy(), 4.5, 1e-8);
}

R2_TEST(restitution_between_boxes_obeys_coefficient) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    cfg.allowSleep = false;
    World world(cfg);
    Body* a = world.add(Body::aabb({-1.0, 0.0}, {0.5, 0.5}, 1.0, 0.5));
    Body* b = world.add(Body::aabb({1.0, 0.0}, {0.5, 0.5}, 1.0, 0.5));
    a->velocity = {2.0, 0.0};
    b->velocity = {-2.0, 0.0};
    for (int i = 0; i < 120; ++i) world.step();

    // Approach speed 4, separation speed must be e * 4 = 2.
    CHECK_NEAR(a->velocity.x, -1.0, 1e-9);
    CHECK_NEAR(b->velocity.x, 1.0, 1e-9);
    CHECK_NEAR(world.totalMomentum().x, 0.0, 1e-10);
}

R2_TEST(three_body_chain_conserves_momentum) {
    WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    cfg.allowSleep = false;
    World world(cfg);
    Body* a = world.add(Body::circle({-2.0, 0.0}, 0.4, 1.0, 0.8, 0.1));
    Body* b = world.add(Body::circle({0.0, 0.0}, 0.4, 2.0, 0.8, 0.1));
    Body* c = world.add(Body::circle({2.0, 0.0}, 0.4, 3.0, 0.8, 0.1));
    a->velocity = {4.0, 0.0};
    b->velocity = {-1.0, 0.0};
    c->velocity = {0.5, 0.0};
    const Vec2 p0 = world.totalMomentum();
    const double ke0 = world.totalKineticEnergy();
    for (int i = 0; i < 600; ++i) world.step();

    // No external forces: momentum conserved exactly (within solver
    // tolerance); kinetic energy never increases (e <= 1, friction >= 0).
    CHECK_NEAR(world.totalMomentum().x, p0.x, 1e-8);
    CHECK_NEAR(world.totalMomentum().y, p0.y, 1e-8);
    CHECK(world.totalKineticEnergy() <= ke0 + 1e-9);
}
