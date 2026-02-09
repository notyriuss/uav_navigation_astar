#pragma once

#include <vector>
#include "Coord.hpp"   // ou o header onde Coord está definido

// Média móvel causal (forward)
std::vector<CoordF> movingAverage(const std::vector<CoordF>& path, int window); 

// Média móvel iterativa
std::vector<CoordF> movingAverageIterativeF(const std::vector<CoordF>& path,
                                            int window,
                                            int iterations);

// Curto-circuito do caminho
std::vector<Coord>
shortcutPath(const std::vector<Coord>& path,
             const std::vector<Coord>& obstacles,
             float robot_radius);

// Reamostra curva
std::vector<CoordF> resampleUniform(
    const std::vector<CoordF>& path,
    float step
);


std::vector<CoordF> densifyPathF(const std::vector<Coord>& path, int n);