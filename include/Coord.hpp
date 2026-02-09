#pragma once
#include <algorithm>
#include <cmath>

struct Coord {
    int r, c;
    
    bool operator<(const Coord& other) const {
        return std::tie(c, r) < std::tie(other.c, other.r);
    }

    bool operator==(const Coord& other) const {
        return r == other.r && c == other.c;
    }
};


struct CoordHash {
    std::size_t operator()(const Coord& c) const {
        return std::hash<int>()(c.r) ^ (std::hash<int>()(c.c) << 1);
    }
};

struct CoordF {
    float r, c;
};

struct Pose2D {
    float x;
    float y;
    float theta;
};
std::vector<Pose2D>
toPosePath(const std::vector<CoordF>& path);

std::vector<CoordF> toCoordF(const std::vector<Coord>& path);
