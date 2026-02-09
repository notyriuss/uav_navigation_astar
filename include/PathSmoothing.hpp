#pragma once

#include <vector>
#include "Coord.hpp"

/**
 * @file PathSmoothing.hpp
 * @brief Collection of geometric path post-processing and smoothing utilities.
 *
 * This module provides discrete and continuous path refinement techniques,
 * commonly used after grid-based planners such as A*.
 * The functions include shortcutting, smoothing, densification and resampling.
 */

/**
 * @brief Applies a simple moving average filter to a sampled path.
 *
 * Smooths the path by averaging neighboring points while keeping
 * the start and goal positions fixed.
 *
 * @param path Input path in floating-point grid coordinates.
 * @param window Half-size of the averaging window.
 * @return Smoothed path with the same number of points.
 */
std::vector<CoordF>
movingAverage(const std::vector<CoordF>& path, int window);

/**
 * @brief Repeatedly applies moving average smoothing.
 *
 * Useful for progressively increasing smoothness without
 * increasing the window size.
 *
 * @param path Input floating-point path.
 * @param window Averaging window size.
 * @param iterations Number of smoothing iterations.
 * @return Smoothed path.
 */
std::vector<CoordF>
movingAverageIterativeF(const std::vector<CoordF>& path,
                        int window,
                        int iterations);

/**
 * @brief Removes unnecessary intermediate nodes from a discrete path.
 *
 * Uses line-of-sight checks in grid space to shortcut the path
 * while respecting obstacle clearance.
 *
 * @param path Discrete grid path.
 * @param obstacles List of obstacle grid coordinates.
 * @param robot_radius Collision radius in grid units.
 * @return Shortened path with preserved start and goal.
 */
std::vector<Coord>
shortcutPath(const std::vector<Coord>& path,
             const std::vector<Coord>& obstacles,
             float robot_radius);

/**
 * @brief Resamples a continuous path with approximately uniform arc-length spacing.
 *
 * This is useful for controllers that assume evenly spaced waypoints.
 *
 * @param path Input floating-point path.
 * @param step Desired arc-length spacing between points.
 * @return Resampled path with uniform spacing.
 */
std::vector<CoordF>
resampleUniform(const std::vector<CoordF>& path,
                float step);

/**
 * @brief Densifies a discrete grid path into a floating-point path.
 *
 * Inserts interpolated points between consecutive grid cells.
 *
 * @param path Discrete grid path.
 * @param n Number of interpolated points per segment.
 * @return Dense floating-point path.
 */
std::vector<CoordF>
densifyPathF(const std::vector<Coord>& path, int n);
