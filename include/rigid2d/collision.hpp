#pragma once

#include "rigid2d/body.hpp"

namespace rigid2d {

// A single contact manifold between two bodies.
//
// Convention: `normal` points from body `a` toward body `b`. The impulse
// solver treats relative velocity vB - vA along +normal as *separating*.
// `penetration` is how deep the shapes overlap (>= 0 while interpenetrating);
// `gap` is the signed separation along the normal, so penetration == -gap
// when the shapes overlap.
struct Contact {
    Body* a = nullptr;
    Body* b = nullptr;
    Vec2 normal;  // unit vector, a -> b
    double penetration = 0.0;
    double gap = 0.0;  // negative while overlapping, positive across a gap
};

// Detects overlap between two bodies. `contactThreshold` extends the
// manifold slightly beyond exact touching (speculative contacts): a pair
// whose gap is below it still reports a contact, with penetration clamped
// at 0 and gap holding the small positive separation. Returns true and
// fills `contact` when the shapes are within the threshold; otherwise
// returns false and leaves `contact` untouched.
bool collide(Body& a, Body& b, Contact& contact,
             double contactThreshold = 0.0);

// ---- Shape-pair primitives. All return false beyond the threshold ----

bool collideCircleCircle(Body& a, Body& b, Contact& out,
                         double contactThreshold = 0.0);
bool collideAabbAabb(Body& a, Body& b, Contact& out,
                     double contactThreshold = 0.0);
bool collideCircleAabb(Body& circle, Body& box, Contact& out,
                       double contactThreshold = 0.0);

}  // namespace rigid2d
