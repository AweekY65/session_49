#pragma once

#include <cmath>

namespace rigid2d {

// Minimal 2D vector. Double precision is used everywhere so that, with a
// fixed integration order, identical inputs always produce bit-identical
// results on the same platform.
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator-() const { return {-x, -y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }

    Vec2& operator+=(const Vec2& o) {
        x += o.x;
        y += o.y;
        return *this;
    }
    Vec2& operator-=(const Vec2& o) {
        x -= o.x;
        y -= o.y;
        return *this;
    }
    Vec2& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }

    bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Vec2& o) const { return !(*this == o); }
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

inline double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

// Scalar cross product z-component of a x b.
inline double cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

inline double lengthSquared(const Vec2& v) { return dot(v, v); }

inline double length(const Vec2& v) { return std::sqrt(lengthSquared(v)); }

inline Vec2 normalized(const Vec2& v) {
    const double len = length(v);
    if (len == 0.0) return {0.0, 0.0};
    return v / len;
}

inline Vec2 clampComponents(const Vec2& v, const Vec2& lo, const Vec2& hi) {
    auto clamp1 = [](double x, double a, double b) {
        return x < a ? a : (x > b ? b : x);
    };
    return {clamp1(v.x, lo.x, hi.x), clamp1(v.y, lo.y, hi.y)};
}

}  // namespace rigid2d
