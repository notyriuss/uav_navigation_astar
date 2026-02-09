#include "Visualizer.hpp"
#include "AStar.hpp"
#include "PathSmoothing.hpp"
#include "Geometry.hpp"

#include <iostream>

/**
 * @brief Parameters controlling the post-processing pipeline.
 */
struct PipelineParams {
    float collision_radius;
    int smooth_window;
    int smooth_iterations;
    float resample_step_raw;
    float resample_step_smooth;
};

/**
 * @brief Executes the complete planning and smoothing pipeline.
 *
 * The pipeline consists of:
 *  1) Global path planning using A*
 *  2) Path shortcutting
 *  3) Uniform resampling
 *  4) Path smoothing via moving average
 *  5) Final uniform resampling
 *
 * @param vis Reference to the visualizer (provides map, start, goal).
 * @param map_width Grid width.
 * @param map_height Grid height.
 * @param params Pipeline configuration parameters.
 * @return Smoothed and resampled path in floating-point coordinates.
 */
std::vector<CoordF>
runPipeline(const Visualizer& vis,
            int map_width,
            int map_height,
            const PipelineParams& params)
{
    //  Global planning (A*) 
    AStar planner(
        vis.get_obstacles(),
        vis.get_start(),
        vis.get_goal(),
        map_width,
        map_height,
        params.collision_radius
    );

    while (!planner.goal_reached() && !planner.no_solution())
        planner.step();

    if (planner.no_solution())
        return {};

    const auto raw_path = planner.path();
    if (raw_path.size() < 2)
        return {};

    //  Path shortcutting 
    const auto shortcut_path =
        shortcutPath(raw_path,
                     vis.get_obstacles(),
                     params.collision_radius);

    const auto shortcutF = toCoordF(shortcut_path);

    //  Densification 
    const auto denseF =
        resampleUniform(shortcutF,
                        params.resample_step_raw);

    //  Smoothing 
    const auto smoothF =
        movingAverageIterativeF(
            denseF,
            params.smooth_window,
            params.smooth_iterations
        );

    // Final resampling
    return resampleUniform(
        smoothF,
        params.resample_step_smooth
    );
}

int main()
{
    // Map width
    constexpr int MAP_W = 40;
    // Map height
    constexpr int MAP_H = 40;
    constexpr int CELL_SIZE = 20;
    // Collision radius considers both robot radius, obstacle radius and security margin
    constexpr float COLLISION_RADIUS = 3.5f;
    // Robot size in comparison with the map cell
    constexpr float DRONE_CELL_FACTOR = 2.0f;

    // Visualization and environment setup 
    Visualizer vis(
        MAP_W,
        MAP_H,
        CELL_SIZE,
        COLLISION_RADIUS,
        DRONE_CELL_FACTOR,
        "../img/drone.png"
    );

    std::cout << "Click to define START, GOAL and obstacles.\n";
    std::cout << "Press ENTER to start planning.\n";
    std::cout << "Press ESC to exit.\n";

    // User input loop
    while (true) {
        int key = cv::waitKey(10);
        if (key == 13)   // ENTER
            break;
        if (key == 27)   // ESC
            return 0;
    }

    // Pipeline configuration
    PipelineParams params {
        .collision_radius = COLLISION_RADIUS,
        .smooth_window = 10,
        .smooth_iterations = 30,
        .resample_step_raw = 0.1f,
        .resample_step_smooth = 0.6f
    };

    // Run planning pipeline
    const auto final_pathF =
        runPipeline(vis, MAP_W, MAP_H, params);

    if (final_pathF.empty()) {
        std::cout << "No valid path found.\n";
        cv::waitKey(0);
        return 0;
    }

    const auto pose_path = toPosePath(final_pathF);

    // Animation
    for (size_t i = 0; i < pose_path.size(); ++i) {
        vis.draw_environment();
        vis.draw_path(final_pathF);

        vis.draw_robot(
            { pose_path[i].y, pose_path[i].x },
            pose_path[i].theta
        );

        if (cv::waitKey(50) == 27) // ESC
            break;
    }

    cv::waitKey(0);
    return 0;
}
