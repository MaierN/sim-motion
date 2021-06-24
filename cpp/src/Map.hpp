#pragma once

#include <vector>

#include "Line.hpp"
#include "Vector2.hpp"

class Map {
   public:
    std::vector<Line> static_obstacles;
    std::vector<Line> dynamic_obstacles;

    Map() {
        static_obstacles.push_back(Line(Vector2(10, 10), Vector2(490, 10)));
        static_obstacles.push_back(Line(Vector2(490, 10), Vector2(490, 740)));
        static_obstacles.push_back(Line(Vector2(490, 740), Vector2(10, 740)));
        static_obstacles.push_back(Line(Vector2(10, 740), Vector2(10, 10)));
        //static_obstacles.push_back(Line(Vector2(200, 200), Vector2(200, 500)));

        dynamic_obstacles.push_back(Line(Vector2(400, 400), Vector2(300, 500)));
        dynamic_obstacles.push_back(Line(Vector2(100, 650), Vector2(400, 650)));
    }

    double min_dist_static_obstacle(const Vector2& p) {
        double min_dist = std::numeric_limits<double>::infinity();
        for (const Line& obstacle : static_obstacles) {
            double distance = obstacle.point_line_distance(p);
            if (distance <= min_dist) min_dist = distance;
        }
        return min_dist;
    }
};
