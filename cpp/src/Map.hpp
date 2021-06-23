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
        static_obstacles.push_back(Line(Vector2(200, 200), Vector2(200, 500)));

        dynamic_obstacles.push_back(Line(Vector2(400, 400), Vector2(300, 500)));
    }
};
