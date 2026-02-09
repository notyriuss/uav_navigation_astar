#pragma once
#include "Coord.hpp"

struct Obstacle {
    Coord pos;
    float radius;   // em células

    bool operator==(const Obstacle& o) const {
        return pos == o.pos && radius == o.radius;
    }
};
