#include "Coord.hpp"


std::vector<Pose2D>
toPosePath(const std::vector<CoordF>& path) {

    std::vector<Pose2D> out;
    if (path.empty()) return out;

    out.reserve(path.size());

    for (size_t i = 0; i < path.size(); ++i) {
        Pose2D p;
        p.x = path[i].c;
        p.y = path[i].r;

        if (i + 1 < path.size()) {
            p.theta = std::atan2(
                path[i+1].r - path[i].r,
                path[i+1].c - path[i].c
            );
        } else if (i > 0) {
            // last point get previous orientation
            p.theta = out.back().theta;
        } else {
            p.theta = 0.f;
        }

        out.push_back(p);
    }

    return out;
}


std::vector<CoordF> toCoordF(const std::vector<Coord>& path) {
    std::vector<CoordF> out;
    out.reserve(path.size());

    for (const auto& p : path)
        out.push_back({ float(p.r), float(p.c) });

    return out;
}