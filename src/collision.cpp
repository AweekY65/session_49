#include "rigid2d/collision.hpp"

#include <algorithm>
#include <cmath>

namespace rigid2d {

namespace {
// Strict manifold threshold: callers using collide() directly get exact
// touching semantics (contact only while overlapping or just kissing).
constexpr double kStrictThreshold = 0.0;
}  // namespace

bool collideCircleCircle(Body& a, Body& b, Contact& out,
                         double contactThreshold) {
    const Vec2 delta = b.position - a.position;
    const double rsum = a.radius + b.radius;
    const double distSq = lengthSquared(delta);
    const double dist = std::sqrt(distSq);

    // Signed gap: negative while overlapping, positive across a gap.
    const double gap = dist - rsum;
    if (gap >= contactThreshold) return false;

    Vec2 n;
    if (dist > 0.0) {
        n = delta / dist;
    } else {
        // Coincident centers: no meaningful direction. Push straight up.
        n = {0.0, 1.0};
    }
    out.a = &a;
    out.b = &b;
    out.normal = n;
    out.gap = gap;
    out.penetration = gap < 0.0 ? -gap : 0.0;
    return true;
}

bool collideAabbAabb(Body& a, Body& b, Contact& out,
                     double contactThreshold) {
    const double dx = b.position.x - a.position.x;
    const double dy = b.position.y - a.position.y;
    const double ox = a.half.x + b.half.x - std::fabs(dx);  // x overlap
    const double oy = a.half.y + b.half.y - std::fabs(dy);  // y overlap

    // Need overlap on BOTH axes (within the contact threshold) to touch.
    if (ox < -contactThreshold || oy < -contactThreshold) return false;

    // Resolve along the axis of least penetration. When the boxes are
    // separated on one axis by a small gap but overlapping on the other,
    // that gapped axis is still the right manifold direction.
    Vec2 n;
    double pen;
    if (ox < oy) {
        n = {dx >= 0.0 ? 1.0 : -1.0, 0.0};
        pen = ox;
    } else {
        n = {0.0, dy >= 0.0 ? 1.0 : -1.0};
        pen = oy;
    }
    out.a = &a;
    out.b = &b;
    out.normal = n;
    out.penetration = pen > 0.0 ? pen : 0.0;
    out.gap = -pen;
    return true;
}

bool collideCircleAabb(Body& circle, Body& box, Contact& out,
                       double contactThreshold) {
    // Nearest point on the box to the circle center.
    const Vec2 nearest = clampComponents(
        circle.position,
        {box.position.x - box.half.x, box.position.y - box.half.y},
        {box.position.x + box.half.x, box.position.y + box.half.y});

    // nearest - center points circle -> box, matching the
    // a(circle) -> b(box) normal convention.
    const Vec2 delta = nearest - circle.position;
    const double distSq = lengthSquared(delta);
    const double dist = std::sqrt(distSq);
    const double gap = dist - circle.radius;

    if (gap >= contactThreshold) return false;

    Vec2 n;
    double pen;
    if (dist > 0.0) {
        // Center outside the box: standard face/corner contact.
        n = delta / dist;
        pen = -gap;
    } else {
        // Center inside the box: contact is on the nearest face; the normal
        // points circle -> that face, so the circle's impulse (-j * n)
        // pushes it out. Penetration is the radius minus the distance to
        // that face.
        const double dLeft =
            circle.position.x - (box.position.x - box.half.x);
        const double dRight =
            (box.position.x + box.half.x) - circle.position.x;
        const double dBottom =
            circle.position.y - (box.position.y - box.half.y);
        const double dTop =
            (box.position.y + box.half.y) - circle.position.y;
        const double minDist =
            std::min(std::min(dLeft, dRight), std::min(dBottom, dTop));
        if (minDist == dLeft) {
            n = {-1.0, 0.0};
        } else if (minDist == dRight) {
            n = {1.0, 0.0};
        } else if (minDist == dBottom) {
            n = {0.0, -1.0};
        } else {
            n = {0.0, 1.0};
        }
        pen = circle.radius - minDist;
    }

    out.a = &circle;
    out.b = &box;
    out.normal = n;  // circle -> box
    out.penetration = pen > 0.0 ? pen : 0.0;
    out.gap = -pen;
    return true;
}

bool collide(Body& a, Body& b, Contact& contact, double contactThreshold) {
    if (a.type == ShapeType::Circle && b.type == ShapeType::Circle) {
        return collideCircleCircle(a, b, contact, contactThreshold);
    }
    if (a.type == ShapeType::AABB && b.type == ShapeType::AABB) {
        return collideAabbAabb(a, b, contact, contactThreshold);
    }
    if (a.type == ShapeType::Circle && b.type == ShapeType::AABB) {
        // collideCircleAabb orders (circle, box) and its normal points
        // circle -> box, already matching a -> b.
        return collideCircleAabb(a, b, contact, contactThreshold);
    }
    // a == AABB, b == Circle: flip the result so the normal points a -> b.
    Contact c;
    if (!collideCircleAabb(b, a, c, contactThreshold)) return false;
    contact.a = &a;
    contact.b = &b;
    contact.normal = -c.normal;
    contact.penetration = c.penetration;
    contact.gap = c.gap;
    return true;
}

}  // namespace rigid2d
