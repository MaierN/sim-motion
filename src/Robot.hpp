#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <random>
#include <vector>

#include "Config.h"
#include "Line.hpp"
#include "Map.hpp"
#include "Vector2.hpp"

class Motion {
   public:
    Vector2 pos_diff;
    double angle_diff;

    Motion(const Vector2& pos_diff, double angle_diff) : pos_diff(pos_diff), angle_diff(angle_diff) {}
    Motion(const Motion& m) : pos_diff(m.pos_diff), angle_diff(m.angle_diff) {}
};

class LidarMeasure {
   public:
    double angle;
    double distance;
    Vector2 point;

    LidarMeasure(double angle, double distance, const Vector2& point)
        : angle(angle), distance(distance), point(point) {}
};

class Robot {
   public:
    Vector2 pos;
    double angle;
    double radius;
    double linear_speed;
    double angular_speed;

    double next_lidar_angle;

    Vector2 last_pos;
    double last_angle;

    Robot()
        : pos(Vector2(100, 100)),
          angle(0),
          radius(20),
          linear_speed(ROBOT_LINEAR_SPEED),
          angular_speed(ROBOT_ANGULAR_SPEED),
          next_lidar_angle(0),
          last_pos(pos),
          last_angle(angle) {}

    Vector2 direction() const {
        return Vector2(cos(angle), sin(angle));
    }
    Vector2 orientation_pos() const {
        return pos + direction() * radius / 2;
    }
    Vector2 left_wheel_pos() const {
        double angle = this->angle - M_PI / 2;
        return pos + Vector2(cos(angle), sin(angle)) * radius;
    }
    Vector2 right_wheel_pos() const {
        double angle = this->angle + M_PI / 2;
        return pos + Vector2(cos(angle), sin(angle)) * radius;
    }
    double size() const {
        return radius * 2;
    }
    LidarMeasure next_measure_lidar(const Map& map) {
        double laser_angle = next_lidar_angle + angle + std::normal_distribution<double>(0, ERROR_LASER_ANGLE)(random_engine);
        Vector2 direction(cos(laser_angle), sin(laser_angle));
        double laser_range = LASER_RANGE;
        Vector2 laser_end_point = pos + direction * laser_range;
        Line laser_segment = Line(pos, laser_end_point);

        double min_dist = laser_range;
        Vector2 min_point = laser_end_point;

        for (const Line& obstacle : map.static_obstacles) {
            LineIntersection li = laser_segment.segment_segment_intersection(obstacle);
            if (li.exists) {
                double distance = (pos - li.p).norm();
                if (distance <= min_dist) {
                    min_dist = distance + std::normal_distribution<double>(0, ERROR_LASER_DISTANCE)(random_engine);
                    min_point = li.p + Vector2(std::normal_distribution<double>(0, ERROR_LASER_DISTANCE)(random_engine),
                                               std::normal_distribution<double>(0, ERROR_LASER_DISTANCE)(random_engine));
                }
            }
        }

        for (const Line& obstacle : map.dynamic_obstacles) {
            LineIntersection li = laser_segment.segment_segment_intersection(obstacle);
            if (li.exists) {
                double distance = (pos - li.p).norm();
                if (distance <= min_dist) {
                    min_dist = distance + std::normal_distribution<double>(0, ERROR_LASER_DISTANCE)(random_engine);;
                    min_point = li.p + Vector2(std::normal_distribution<double>(0, ERROR_LASER_DISTANCE)(random_engine),
                                               std::normal_distribution<double>(0, ERROR_LASER_DISTANCE)(random_engine));
                }
            }
        }

        double old_lidar_angle = next_lidar_angle;
        next_lidar_angle += 2 * M_PI / N_LASER;
        return LidarMeasure(old_lidar_angle, min_dist, min_point);
    }

    Motion get_motion_update() {
        Vector2 pos_diff = (pos - last_pos).rotated(-last_angle);
        double angle_diff = angle - last_angle;

        last_pos = pos;
        last_angle = angle;

        return Motion(pos_diff, angle_diff);
    }
};
