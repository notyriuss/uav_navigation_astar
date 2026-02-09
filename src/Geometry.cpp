#include "Geometry.hpp"
#include <cmath>

float dist(const Coord& a, const Coord& b) {
    return std::hypot(float(a.r - b.r),
                      float(a.c - b.c));
}

float dist(const CoordF& a, const CoordF& b) {
    return std::hypot(a.r - b.r, a.c - b.c);
}

