#include "AStar.hpp"
#include <cmath>
#include <algorithm>

AStar::AStar(const std::vector<Coord>& obstacles,
             Coord start,
             Coord goal,
             int width,
             int height,
             float robot_radius)
: obstacles_(obstacles),
  start_(start),
  goal_(goal),
  width_(width),
  height_(height),
  robot_radius_(robot_radius)
{
    AStarNode n0;
    n0.state = start_;
    n0.g = 0.0f;
    n0.h = heuristic(start_);
    n0.f = n0.g + n0.h;
    n0.path.push_back(start_);

    open_list_.push(n0);
}

float AStar::heuristic(const Coord& s) const {
    return std::hypot(float(s.r - goal_.r),
                      float(s.c - goal_.c));
}

bool AStar::collides(const Coord& s) const {
    for (const auto& obs : obstacles_) {
        float d = std::hypot(float(s.r - obs.r),
                             float(s.c - obs.c));
        if (d <= robot_radius_)
            return true;
    }
    return false;
}

std::vector<AStarNode>
AStar::generate_neighbors(const AStarNode& current) {
    static const std::vector<Coord> moves = {
        {-1,  0}, { 1,  0}, { 0, -1}, { 0,  1},
        {-1, -1}, {-1,  1}, { 1, -1}, { 1,  1}
    };

    std::vector<AStarNode> neighbors;

    for (const auto& m : moves) {
        Coord next{
            current.state.r + m.r,
            current.state.c + m.c
        };

        // limites do grid
        if (next.r < 0 || next.r >= height_ ||
            next.c < 0 || next.c >= width_)
            continue;

        // colisão circular
        if (collides(next))
            continue;

        AStarNode node;
        node.state = next;

        float step_cost = std::hypot(float(m.r), float(m.c));
        node.g = current.g + step_cost;
        node.h = heuristic(next);
        node.f = node.g + node.h;

        node.path = current.path;
        node.path.push_back(next);

        neighbors.push_back(node);
    }

    return neighbors;
}

bool AStar::no_solution() const{
    return no_solution_;
}


Coord AStar::step() {
    if (open_list_.empty()) {
        no_solution_ = true;
        return {-1, -1}; // estado inválido
    }


    AStarNode current = open_list_.top();
    open_list_.pop();

    if (current.state == goal_) {
        goal_reached_ = true;
        solution_path_ = current.path;
        return current.state;
    }

    auto it = closed_list_.find(current.state);
    if (it != closed_list_.end() && it->second <= current.g)
        return current.state;

    closed_list_[current.state] = current.g;

    for (auto& n : generate_neighbors(current))
        open_list_.push(n);

    return current.state;
}

bool AStar::goal_reached() const {
    return goal_reached_;
}

std::vector<Coord> AStar::path() const {
    return solution_path_;
}

