#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <random>

#include "Line.hpp"
#include "Robot.hpp"
#include "Vector2.hpp"
#include "config.h"

/**
 * Particle for the Monte Carlo simulation representing a potential position for the robot
 */
class Particle {
   public:
    Vector2 pos;
    double angle;
    double weight;

    Particle() : weight(1) {
        pos = Vector2(std::normal_distribution<double>(100, ERROR_PARTICLE_INITIAL_POS)(random_engine), std::normal_distribution<double>(100, ERROR_PARTICLE_INITIAL_POS)(random_engine));
        angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(random_engine);
    }
    Particle(const Particle& p) : pos(p.pos), angle(p.angle), weight(p.weight) {
        pos += Vector2(std::normal_distribution<double>(0, OFFSET_PARTICLE_RESAMPLE_POS)(random_engine), std::normal_distribution<double>(0, OFFSET_PARTICLE_RESAMPLE_POS)(random_engine));
        angle += std::normal_distribution<double>(0, OFFSET_PARTICLE_RESAMPLE_ANGLE)(random_engine);
    }

    void apply_motion(Motion m) {
        pos += m.pos_diff.rotated(angle);
        angle += m.angle_diff;
    }
    LidarMeasure sensor_measure(const Map& map, double measure_angle) const {
        double laser_angle = measure_angle + angle;
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
                    min_dist = distance;
                    min_point = li.p;
                }
            }
        }

        return LidarMeasure(measure_angle, min_dist, min_point);
    }
};
