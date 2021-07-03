#pragma once

#include <cmath>
#include <iostream>

/**
 * Vector in 2-dimensional space
 */
class Vector2 {
   public:
    double x;
    double y;

    Vector2() : x(0), y(0) {}
    Vector2(double x, double y) : x(x), y(y) {}
    Vector2(const Vector2& v) : x(v.x), y(v.y) {}

    double norm() const {
        return sqrt(x * x + y * y);
    }

    Vector2 operator+(const Vector2& v) const {
        return Vector2(x + v.x, y + v.y);
    }
    Vector2 operator-(const Vector2& v) const {
        return Vector2(x - v.x, y - v.y);
    }
    Vector2 operator*(const double f) const {
        return Vector2(x * f, y * f);
    }
    Vector2 operator/(const double f) const {
        return Vector2(x / f, y / f);
    }
    Vector2& operator+=(const Vector2& v) {
        x += v.x;
        y += v.y;
        return *this;
    }
    Vector2& operator-=(const Vector2& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }
    Vector2& operator*=(const double& f) {
        x *= f;
        y *= f;
        return *this;
    }
    Vector2& operator/=(const double& f) {
        x /= f;
        y /= f;
        return *this;
    }

    Vector2 normalized() const {
        return *this / this->norm();
    }
    Vector2 moved_towards(const Vector2& target, double max_dist) const {
        Vector2 delta = target - *this;
        if (delta.norm() <= max_dist) return target;
        return *this + delta.normalized() * max_dist;
    }
    Vector2 rotated(double angle) const {
        return Vector2(x * cos(angle) + y * -sin(angle), x * sin(angle) + y * cos(angle));
    }
};

std::ostream& operator<<(std::ostream& os, const Vector2& v) {
    return os << "v(" << v.x << " ; " << v.y << ")";
}
