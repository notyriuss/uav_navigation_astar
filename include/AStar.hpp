#pragma once
#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "Coord.hpp"

struct AStarNode {
    Coord state;
    float g, h, f;
    std::vector<Coord> path;
};

struct CompareAStarNode {
    bool operator()(const AStarNode& a,
                    const AStarNode& b) const {
        return a.f > b.f;
    }
};

class AStar {
public:
    AStar(const std::vector<Coord>& obstacles,
          Coord start,
          Coord goal,
          int width,
          int height,
          float robot_radius);

    Coord step();
    bool goal_reached() const;
    std::vector<Coord> path() const;
    bool no_solution() const;

private:
    bool no_solution_ = false;
    float heuristic(const Coord& s) const;
    bool collides(const Coord& s) const;
    std::vector<AStarNode> generate_neighbors(const AStarNode& current);

    std::priority_queue<
        AStarNode,
        std::vector<AStarNode>,
        CompareAStarNode
    > open_list_;

    std::unordered_map<Coord, float, CoordHash> closed_list_;

    std::vector<Coord> obstacles_;
    std::vector<Coord> solution_path_;

    Coord start_, goal_;
    int width_, height_;
    float robot_radius_;
    bool goal_reached_ = false;
};

