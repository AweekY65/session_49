#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "rigid2d/vec2.hpp"

namespace rigid2d {

enum class ShapeType : std::uint8_t { Circle, AABB };

// A rigid body. Bodies are non-rotating: only position, linear velocity and
// mass are simulated. A circle is described by radius; an AABB by half
// extents on each axis (axis-aligned, no rotation).
//
// Static bodies have infinite mass: inverseMass == 0, they never move and
// never gain velocity from collisions.
struct Body {
    ShapeType type;
    Vec2 position;   // center of mass
    Vec2 velocity;
    Vec2 force;      // accumulated external force for the current step
    double radius = 0.0;   // circle only
    Vec2 half;             // AABB only: half width / half height
    double mass = 0.0;
    double inverseMass = 0.0;
    double restitution = 0.0;  // coefficient of restitution, clamped to [0,1]
    double friction = 0.0;     // Coulomb friction coefficient, clamped >= 0
    bool isStatic = false;
    bool asleep = false;

    // Timer of seconds the body has spent (almost) at rest; >= sleepTime puts
    // it to sleep. Dynamic bodies only.
    double restTimer = 0.0;

    // Optional label, useful in test diagnostics.
    std::string name;

    Body(ShapeType t, const Vec2& pos) : type(t), position(pos) {}

    // ---- Factory functions; these validate the inputs explicitly ----

    static Body circle(const Vec2& pos, double r, double m,
                       double restitution_ = 0.0, double friction_ = 0.0) {
        if (!(r > 0.0) || !std::isfinite(r)) {
            throw std::invalid_argument(
                "circle radius must be finite and positive");
        }
        Body b(ShapeType::Circle, pos);
        b.radius = r;
        b.half = {r, r};
        b.setMass(m);
        b.setRestitution(restitution_);
        b.setFriction(friction_);
        return b;
    }

    static Body aabb(const Vec2& pos, const Vec2& halfExtents, double m,
                     double restitution_ = 0.0, double friction_ = 0.0) {
        if (!(halfExtents.x > 0.0) || !(halfExtents.y > 0.0) ||
            !std::isfinite(halfExtents.x) || !std::isfinite(halfExtents.y)) {
            throw std::invalid_argument(
                "AABB half extents must be finite and positive");
        }
        Body b(ShapeType::AABB, pos);
        b.half = halfExtents;
        b.setMass(m);
        b.setRestitution(restitution_);
        b.setFriction(friction_);
        return b;
    }

    // Zero or negative mass means *invalid* for a dynamic body; use
    // makeStatic() for immovable geometry. Infinite mass is also rejected to
    // keep dynamic bodies well defined. Use finite positive masses or static.
    void setMass(double m) {
        if (!(m > 0.0) || !std::isfinite(m)) {
            throw std::invalid_argument(
                "dynamic body mass must be finite and positive; "
                "use makeStatic() for immovable bodies");
        }
        mass = m;
        inverseMass = 1.0 / m;
        isStatic = false;
    }

    void makeStatic() {
        mass = 0.0;
        inverseMass = 0.0;
        isStatic = true;
        velocity = {0.0, 0.0};
        force = {0.0, 0.0};
        asleep = false;
        restTimer = 0.0;
    }

    // Restitution outside [0,1] is clamped, not rejected: values > 1 would
    // inject energy and < 0 is physically meaningless.
    void setRestitution(double e) {
        if (!std::isfinite(e)) e = 0.0;
        restitution = e < 0.0 ? 0.0 : (e > 1.0 ? 1.0 : e);
    }

    // Negative friction is meaningless and is clamped to 0.
    void setFriction(double f) {
        if (!std::isfinite(f) || f < 0.0) f = 0.0;
        friction = f;
    }

    bool movable() const { return !isStatic && inverseMass > 0.0; }

    // Wakes a sleeping dynamic body and resets its rest timer.
    void wake() {
        if (isStatic) return;
        asleep = false;
        restTimer = 0.0;
    }
};

}  // namespace rigid2d
