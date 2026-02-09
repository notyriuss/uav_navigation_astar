#include "Visualizer.hpp"
#include <iostream>

Visualizer::Visualizer(int width, int height, int cell_size,
                       float collision_radius,
                       float drone_cell_factor,
                       const std::string& drone_png)
: width_(width), height_(height), cell_(cell_size),
  collision_radius_(collision_radius), drone_cell_factor_(drone_cell_factor)
{
    base_img_ = cv::Mat(height_*cell_, width_*cell_, CV_8UC3);
    img_ = base_img_.clone();

    drone_sprite_ = cv::imread(drone_png, cv::IMREAD_UNCHANGED);
    if (drone_sprite_.empty()) {
        throw std::runtime_error("Erro ao carregar sprite do drone");
    }

    cv::namedWindow("Map");
    cv::setMouseCallback("Map", on_mouse, this);
    draw_environment();

}

Visualizer::~Visualizer() {
    cv::destroyAllWindows();
}

void Visualizer::draw_grid() {
    const cv::Scalar grid_color(120,120,120);
    for (int i = 0; i <= width_; ++i)
        cv::line(img_, {i*cell_,0}, {i*cell_,height_*cell_}, grid_color);

    for (int j = 0; j <= height_; ++j)
        cv::line(img_, {0,j*cell_}, {width_*cell_,j*cell_}, grid_color);
}

void Visualizer::draw_environment() {
    img_.setTo(cv::Scalar(0,0,0));

    // Obstáculos (ainda inteiros)
    for (auto& o : obstacles_) {
        cv::rectangle(img_,
            {o.c*cell_, o.r*cell_},
            {(o.c+1)*cell_-1, (o.r+1)*cell_-1},
            cv::Scalar(255,255,255), cv::FILLED);

        cv::Point center(o.c*cell_ + cell_/2,
                         o.r*cell_ + cell_/2);
        cv::circle(img_, center,
                   collision_radius_ * cell_,
                   cv::Scalar(100,100,100), 1);
    }

    if (start_set_) {
        cv::circle(img_,
            {static_cast<int>(start_.c * cell_),
             static_cast<int>(start_.r * cell_)},
            cell_/3, cv::Scalar(0,255,0), cv::FILLED);
    }

    if (goal_set_) {
        cv::circle(img_,
            {static_cast<int>(goal_.c * cell_),
             static_cast<int>(goal_.r * cell_)},
            cell_/3, cv::Scalar(0,0,255), cv::FILLED);
    }

    draw_grid();
    cv::imshow("Map", img_);
}


void Visualizer::draw_path(const std::vector<CoordF>& path) {
    for (size_t i = 1; i < path.size(); ++i) {
        cv::Point p1(
            static_cast<int>(path[i-1].c * cell_),
            static_cast<int>(path[i-1].r * cell_)
        );
        cv::Point p2(
            static_cast<int>(path[i].c * cell_),
            static_cast<int>(path[i].r * cell_)
        );
        cv::line(img_, p1, p2, cv::Scalar(0,255,255), 2);
    }
}

void Visualizer::draw_robot(const CoordF& pos, float theta) {
    cv::Mat frame = img_.clone();

    int drone_size_px = static_cast<int>(cell_ * drone_cell_factor_);
    drone_size_px = std::max(2, drone_size_px);

    cv::Mat drone_resized;
    cv::resize(drone_sprite_, drone_resized,
               cv::Size(drone_size_px, drone_size_px),
               0, 0, cv::INTER_AREA);

    cv::Mat rot;
    cv::Point2f center(drone_resized.cols / 2.f,
                       drone_resized.rows / 2.f);

    auto M = cv::getRotationMatrix2D(center, -theta * 180.0 / M_PI, 1.0);
    cv::warpAffine(drone_resized, rot, M, drone_resized.size(),
                   cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

    // posição usando float multiplicado pelo tamanho da célula
    int x = static_cast<int>(pos.c * cell_ - rot.cols / 2);
    int y = static_cast<int>(pos.r * cell_ - rot.rows / 2);

    for (int i = 0; i < rot.rows; ++i) {
        for (int j = 0; j < rot.cols; ++j) {
            int fx = x + j;
            int fy = y + i;
            if (fx < 0 || fy < 0 || fx >= frame.cols || fy >= frame.rows)
                continue;

            cv::Vec4b px = rot.at<cv::Vec4b>(i,j);
            float alpha = px[3] / 255.f;
            if (alpha <= 0.f) continue;

            cv::Vec3b& bg = frame.at<cv::Vec3b>(fy, fx);
            for (int k = 0; k < 3; ++k)
                bg[k] = static_cast<uchar>(alpha * px[k] + (1.f - alpha) * bg[k]);
        }
    }

    cv::imshow("Map", frame);
}



bool Visualizer::collides(const Coord& cell) const {
    for (auto& obs : obstacles_) {
        float d = std::hypot(cell.r - obs.r,
                             cell.c - obs.c);
        if (d <= collision_radius_)
            return true;
    }
    return false;
}

std::vector<Coord> Visualizer::get_obstacles() const {
    return std::vector<Coord>(obstacles_.begin(), obstacles_.end());
}

Coord Visualizer::get_start() const { return start_; }
Coord Visualizer::get_goal()  const { return goal_; }

void Visualizer::on_mouse(int event, int x, int y, int, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto* v = static_cast<Visualizer*>(userdata);

    Coord c{y / v->cell_, x / v->cell_};

    if (!v->start_set_) { v->start_ = c; v->start_set_ = true; }
    else if (!v->goal_set_) { v->goal_ = c; v->goal_set_ = true; }
    else v->obstacles_.insert(c);

    v->draw_environment();
}

