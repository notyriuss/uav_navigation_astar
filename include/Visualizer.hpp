#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_set>
#include <cmath>
#include "Coord.hpp"


class Visualizer {
public:
    Visualizer(int width, int height, int cell_size,
               float robot_radius,
               float drone_cell_factor,
               const std::string& drone_png);

    ~Visualizer();

    void draw_environment();
    //void draw_path(const std::vector<Coord>& path);
    //void draw_robot(const Coord& pos, float theta);
    void draw_path(const std::vector<CoordF>& pathF);
    void draw_robot(const CoordF& posF, float theta);
    std::vector<Coord> get_obstacles() const;
    Coord get_start() const;
    Coord get_goal() const;

private:
    static void on_mouse(int event, int x, int y, int flags, void* userdata);

    bool collides(const Coord& cell) const;
    void draw_grid();

    int width_, height_, cell_;
    float collision_radius_, drone_cell_factor_;

    cv::Mat base_img_;
    cv::Mat img_;
    cv::Mat drone_sprite_;

    std::unordered_set<Coord, CoordHash> obstacles_;
    Coord start_, goal_;
    bool start_set_ = false;
    bool goal_set_  = false;
};

