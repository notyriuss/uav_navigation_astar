#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_set>
#include <cmath>
#include "Coord.hpp"

/**
 * @class Visualizer
 * @brief 2D grid-based visualizer for mobile robot navigation.
 *
 * This class provides a lightweight OpenCV-based visualization layer for
 * grid environments. It supports interactive definition of:
 *  - start position
 *  - goal position
 *  - obstacles
 *
 * It also handles rendering of:
 *  - grid cells
 *  - obstacle collision radius
 *  - planned paths
 *  - robot pose using a rotated sprite
 *
 * All map coordinates are represented in grid units (row, column) and
 * internally converted to pixel coordinates for visualization.
 *
 * This class is strictly a visualization and input utility:
 * it does NOT perform planning or control.
 */
class Visualizer {
public:
    /**
     * @brief Constructs a Visualizer instance.
     *
     * @param width Number of grid cells in the horizontal direction.
     * @param height Number of grid cells in the vertical direction.
     * @param cell_size Pixel size of each grid cell.
     * @param robot_radius Collision radius in grid-cell units.
     * @param drone_cell_factor Scaling factor for the robot sprite relative to a cell.
     * @param drone_png Path to a PNG image with alpha channel for the robot sprite.
     *
     * @throws std::runtime_error if the sprite image cannot be loaded.
     */
    Visualizer(int width, int height, int cell_size,
               float robot_radius,
               float drone_cell_factor,
               const std::string& drone_png);

    /**
     * @brief Destroys the Visualizer and closes all OpenCV windows.
     */
    ~Visualizer();

    /**
     * @brief Draws the entire environment.
     *
     * This includes:
     *  - background
     *  - obstacles
     *  - collision radius
     *  - start and goal markers
     *  - grid overlay
     */
    void draw_environment();

    /**
     * @brief Draws a planned path represented in floating-point grid coordinates.
     *
     * @param pathF Sequence of positions in (row, column) grid coordinates.
     */
    void draw_path(const std::vector<CoordF>& pathF);

    /**
     * @brief Draws the robot sprite at a given pose.
     *
     * @param posF Robot position in grid coordinates.
     * @param theta Robot orientation in radians (counterclockwise).
     */
    void draw_robot(const CoordF& posF, float theta);

    /**
     * @brief Returns the list of obstacle cells.
     */
    std::vector<Coord> get_obstacles() const;

    /**
     * @brief Returns the start cell.
     */
    Coord get_start() const;

    /**
     * @brief Returns the goal cell.
     */
    Coord get_goal() const;

private:
    /**
     * @brief Mouse callback for interactive map editing.
     *
     * Mouse left-click behavior:
     *  - first click: set start
     *  - second click: set goal
     *  - subsequent clicks: insert obstacles
     */
    static void on_mouse(int event, int x, int y, int flags, void* userdata);

    /**
     * @brief Checks if a grid cell is within the collision radius of any obstacle.
     */
    bool collides(const Coord& cell) const;

    /**
     * @brief Draws the grid overlay.
     */
    void draw_grid();

    int width_;                 ///< Grid width (number of columns)
    int height_;                ///< Grid height (number of rows)
    int cell_;                  ///< Cell size in pixels

    float collision_radius_;    ///< Collision radius in grid units
    float drone_cell_factor_;   ///< Sprite scaling factor relative to cell size

    cv::Mat base_img_;          ///< Base image buffer
    cv::Mat img_;               ///< Current environment image
    cv::Mat drone_sprite_;      ///< RGBA robot sprite

    std::unordered_set<Coord, CoordHash> obstacles_; ///< Set of obstacle cells

    Coord start_;               ///< Start cell
    Coord goal_;                ///< Goal cell
    bool start_set_ = false;    ///< Whether start has been defined
    bool goal_set_  = false;    ///< Whether goal has been defined
};
