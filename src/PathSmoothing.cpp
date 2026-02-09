#include "PathSmoothing.hpp"
#include "Geometry.hpp"


// Checks if a grid cell collides with any obstacle considering robot radius
bool cellCollision(const Coord& cell,
                   const std::vector<Coord>& obstacles,
                   float robot_radius)
{
    for (const auto& obs : obstacles)
        if (dist(cell, obs) <= robot_radius)
            return true;
    return false;
}


// Bresenham line traversal between two grid cells
std::vector<Coord>
bresenhamCells(const Coord& a, const Coord& b)
{
    std::vector<Coord> out;

    int x0 = a.c, y0 = a.r;
    int x1 = b.c, y1 = b.r;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        out.push_back({y0, x0});
        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }

    return out;
}


// Checks if the straight grid path between two points is collision-free
bool gridPathFree(const Coord& a,
                  const Coord& b,
                  const std::vector<Coord>& obstacles,
                  float robot_radius)
{
    for (const auto& c : bresenhamCells(a, b))
        if (cellCollision(c, obstacles, robot_radius))
            return false;
    return true;
}


// Shortcut path by removing unnecessary intermediate nodes
std::vector<Coord>
shortcutPath(const std::vector<Coord>& path,
             const std::vector<Coord>& obstacles,
             float robot_radius)
{
    if (path.size() < 2)
        return path;

    std::vector<Coord> result;
    size_t i = 0;

    result.push_back(path[i]);

    while (i < path.size() - 1) {
        size_t j = path.size() - 1;

        while (j > i + 1) {
            if (gridPathFree(path[i], path[j],
                             obstacles, robot_radius))
                break;
            --j;
        }

        result.push_back(path[j]);
        i = j;
    }

    return result;
}


// Moving average smoothing keeping endpoints fixed
std::vector<CoordF> movingAverage(const std::vector<CoordF>& path, int window)
{
    if (path.size() < 3)
        return path;

    std::vector<CoordF> out = path;

    // keep start fixed
    out.front() = path.front();
    // keep end fixed
    out.back() = path.back();

    size_t N = path.size();
    for (size_t i = 1; i < N - 1; ++i) {
        float sr = 0.f, sc = 0.f;
        int cnt = 0;

        // centered window, excluding endpoints
        size_t start = (i < window) ? 1 : i - window;
        size_t end   = std::min(i + window, N - 2);

        for (size_t k = start; k <= end; ++k) {
            sr += path[k].r;
            sc += path[k].c;
            ++cnt;
        }

        out[i].r = sr / cnt;
        out[i].c = sc / cnt;
    }

    return out;
}


// Applies moving average multiple times
std::vector<CoordF>
movingAverageIterativeF(const std::vector<CoordF>& path,
                        int window,
                        int iterations)
{
    std::vector<CoordF> cur = path;

    for (int i = 0; i < iterations; ++i)
        cur = movingAverage(cur, window);

    return cur;
}


// Resamples a continuous path with uniform arc-length spacing
std::vector<CoordF>
resampleUniform(const std::vector<CoordF>& path,
                float step)
{
    if (path.size() < 2 || step <= 0.f)
        return path;

    // cumulative arc length
    std::vector<float> s(path.size(), 0.f);
    for (size_t i = 1; i < path.size(); ++i)
        s[i] = s[i - 1] + dist(path[i - 1], path[i]);

    float total = s.back();
    int M = std::max(2, int(std::round(total / step)));

    std::vector<CoordF> out;
    out.reserve(M);

    // fixed start
    out.push_back(path.front());

    size_t seg = 1;
    for (int i = 1; i < M - 1; ++i) {
        float target = i * total / (M - 1);

        while (seg < s.size() - 1 && s[seg] < target)
            ++seg;

        float t = (target - s[seg - 1]) / (s[seg] - s[seg - 1]);

        CoordF p;
        p.r = path[seg - 1].r + t * (path[seg].r - path[seg - 1].r);
        p.c = path[seg - 1].c + t * (path[seg].c - path[seg - 1].c);
        out.push_back(p);
    }

    // fixed end
    out.push_back(path.back());
    return out;
}


// Converts a discrete grid path into a denser floating-point path
std::vector<CoordF>
densifyPathF(const std::vector<Coord>& path, int n)
{
    std::vector<CoordF> denseF;
    if (path.size() < 2 || n <= 0)
        return denseF;

    denseF.push_back({float(path[0].r), float(path[0].c)});

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Coord& start = path[i];
        const Coord& end   = path[i + 1];

        for (int k = 1; k <= n; ++k) {
            double t = double(k) / (n + 1);
            float r = float(start.r + t * (end.r - start.r));
            float c = float(start.c + t * (end.c - start.c));
            denseF.push_back({r, c});
        }

        denseF.push_back({float(end.r), float(end.c)});
    }

    return denseF;
}
