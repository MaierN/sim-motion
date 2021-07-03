#pragma once

#include <cmath>

#include "Line.hpp"
#include "Vector2.hpp"

/**
 * Result of the intersection of two lines (either empty or a single point)
 */
class LineIntersection {
   public:
    Vector2 p;
    bool exists;

    LineIntersection(const Vector2& p, const bool exists) : p(p), exists(exists) {}
};

/**
 * Line in a 2-dimensional space
 */
class Line {
   public:
    Vector2 p1;
    Vector2 p2;

    Line(const Vector2& p1, const Vector2& p2) : p1(p1), p2(p2) {}

    LineIntersection segment_segment_intersection(const Line& l) {
        LineIntersection li = line_line_intersection(l);
        if (li.exists) li.exists = point_in_segment(li.p) && l.point_in_segment(li.p);
        return li;
    }

    LineIntersection line_line_intersection(const Line& l) const {
        double det = (p1.x - p2.x) * (l.p1.y - l.p2.y) - (p1.y - p2.y) * (l.p1.x - l.p2.x);
        if (abs(det) <= 1e-3) return LineIntersection(Vector2(0, 0), false);
        double px = ((p1.x * p2.y - p1.y * p2.x) * (l.p1.x - l.p2.x) - (p1.x - p2.x) * (l.p1.x * l.p2.y - l.p1.y * l.p2.x)) / det;
        double py = ((p1.x * p2.y - p1.y * p2.x) * (l.p1.y - l.p2.y) - (p1.y - p2.y) * (l.p1.x * l.p2.y - l.p1.y * l.p2.x)) / det;
        return LineIntersection(Vector2(px, py), true);
    }

    bool point_in_segment(const Vector2& p) const {
        Vector2 direction = p2 - p1;
        double angle = atan2(direction.y, direction.x);
        Vector2 r_l1 = p1.rotated(-angle);
        Vector2 r_l2 = p2.rotated(-angle);
        Vector2 r_p = p.rotated(-angle);
        return (r_p.x >= r_l1.x && r_p.x <= r_l2.x) || (r_p.x <= r_l1.x && r_p.x >= r_l2.x);
    }

    double point_line_distance(const Vector2& p) const {
        Vector2 direction = p2 - p1;
        double angle = atan2(direction.y, direction.x);
        Vector2 r_l1 = p1.rotated(-angle);
        Vector2 r_l2 = p2.rotated(-angle);
        Vector2 r_p = p.rotated(-angle);

        if (point_in_segment(p)) {
            return abs(r_p.y - r_l1.y);
        } else {
            return std::min((r_p - r_l1).norm(), (r_p - r_l2).norm());
        }
    }
};
