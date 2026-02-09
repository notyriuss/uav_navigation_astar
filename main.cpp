#include "Visualizer.hpp"
#include "AStar.hpp"
#include "PathSmoothing.hpp"
#include "Geometry.hpp"

#include <iostream>


std::vector<CoordF>
runPipeline(
    const std::vector<Coord>& raw_path,
    const std::vector<Coord>& obstacles,
    float robot_radius,
    int densify_n,
    int smooth_window,
    int smooth_iterations,
    float resample_step,
    float resample_step_smooth
) {
    if (raw_path.size() < 2)
        return {};

    //  Path short-circuit
    auto shortcut_path =
        shortcutPath(raw_path, obstacles, robot_radius);

    auto shortcutF = toCoordF(shortcut_path);

    // Path resampling
    auto denseF_path = resampleUniform(shortcutF, resample_step);

    // Path smoothing by moving averages
    auto smoothF_path =
        movingAverageIterativeF(
            denseF_path,
            smooth_window,
            smooth_iterations
        );

    // Uniform resampling of the path
    return resampleUniform(smoothF_path, resample_step_smooth);
}




int main() {

    constexpr int W = 40;
    constexpr int H = 40;
    constexpr int CELL_SZ = 20;
    constexpr float ROBOT_RADIUS = 3.5f;
    constexpr float DRONE_CELL_FACTOR = 2.f;

    Visualizer vis(
        W, H,
        CELL_SZ,
        ROBOT_RADIUS,
        DRONE_CELL_FACTOR,
        "../img/drone.png"
    );

    std::cout << "Clique para definir START, GOAL e obstáculos\n";
    std::cout << "Pressione ENTER para iniciar\n";

    while (true) {
        int k = cv::waitKey(10);
        if (k == 13) break;
        if (k == 27) return 0;
    }

    // A* algorithm

    AStar planner(
        vis.get_obstacles(),
        vis.get_start(),
        vis.get_goal(),
        W, H,
        ROBOT_RADIUS
    );

    while (!planner.goal_reached() &&
           !planner.no_solution())
        planner.step();

    if (planner.no_solution()) {
        std::cout << "Nenhum caminho encontrado\n";
        cv::waitKey(0);
        return 0;
    }

    // ================= PIPELINE =================

    const auto raw_path = planner.path();

    const auto final_pathF = runPipeline(
        raw_path,
        vis.get_obstacles(),
        ROBOT_RADIUS,
        /* densify_n         */ 100,
        /* smooth_window     */ 10,
        /* smooth_iterations */ 30,
        /* resample_step_shortcut     */ 0.1f,
        /* resample_step_smooth     */ 0.6f
    );

    const auto pose_path = toPosePath(final_pathF);

    // ANIMATION

    for (size_t i = 0; i < pose_path.size(); ++i) {
        vis.draw_environment();
        vis.draw_path(final_pathF);

        vis.draw_robot(
            { pose_path[i].y, pose_path[i].x },
            pose_path[i].theta
        );

        if (i + 1 < pose_path.size()) {
            if (cv::waitKey(50) == 27)
                break;
        }
    }

    cv::waitKey(0);
    return 0;
}
