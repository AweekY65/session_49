// Minimal headless example: two circles and a static ground simulated for
// a few fixed steps, printing state to the terminal. No rendering.
//
// Build:  g++ -std=c++17 -Iinclude examples/headless_demo.cpp \
//            src/collision.cpp src/world.cpp -o demo
// Run:    ./demo

#include <cstdio>

#include "rigid2d/world.hpp"

int main() {
    using namespace rigid2d;

    WorldConfig cfg;
    cfg.gravity = {0.0, -9.81};
    World world(cfg);

    // Static ground: a wide AABB centered 1 m below the surface at y = 0.
    Body* ground = world.add(Body::aabb({0.0, -1.0}, {10.0, 1.0}, 1.0));
    ground->makeStatic();

    // Two bouncy circles dropped from different heights.
    Body* a = world.add(Body::circle({-0.6, 3.0}, 0.3, 1.0, 0.7, 0.4));
    Body* b = world.add(Body::circle({0.6, 5.0}, 0.3, 2.0, 0.5, 0.4));

    for (int i = 0; i < 240; ++i) {  // 4 simulated seconds at 60 Hz
        world.step();
        if (i % 20 == 0) {
            std::printf(
                "t=%4.2f  a(pos %.2f,%5.2f vel %6.2f,%6.2f)  "
                "b(pos %.2f,%5.2f vel %6.2f,%6.2f)  KE=%.3f P=(%.3f,%.3f)\n",
                (i + 1) / 60.0,
                a->position.x, a->position.y, a->velocity.x, a->velocity.y,
                b->position.x, b->position.y, b->velocity.x, b->velocity.y,
                world.totalKineticEnergy(), world.totalMomentum().x,
                world.totalMomentum().y);
        }
    }
    return 0;
}
