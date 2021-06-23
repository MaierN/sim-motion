#pragma once

#include <cmath>
#include <iostream>

class Vector2 {
   public:
    float x;
    float y;

    Vector2(float x, float y) : x(x), y(y) {}

    float norm() const {
        return sqrt(x * x + y * y);
    }

    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }
    Vector2 operator*(const float f) const {
        return Vector2(x * f, y * f);
    }
    Vector2 operator/(const float f) const {
        return Vector2(x / f, y / f);
    }

    Vector2 normalized() const {
        return *this / this->norm();
    }
    Vector2 moved_towards(const Vector2& target, float max_dist) const {
        Vector2 delta = target - *this;
        if (delta.norm() <= max_dist) return target;
        return *this + delta.normalized() * max_dist;
    }
    Vector2 rotated(float angle) const {
        return Vector2(x * cos(angle) + y * -sin(angle),
                       x * sin(angle) + y * cos(angle));
    }
};

std::ostream& operator<<(std::ostream& os, Vector2 const& o) {
    return os << "v(" << o.x << " ; " << o.y << ")";
}
