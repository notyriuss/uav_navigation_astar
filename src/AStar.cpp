#include "AStar.hpp"
#include <cmath>
#include <algorithm>

/**
 * @brief Constructs an A* planner.
 *
 * Initializes the open list with the starting node and computes
 * its heuristic cost to the goal.
 *
 * @param obstacles List of obstacle positions in grid coordinates.
 * @param start Starting grid coordinate.
 * @param goal Goal grid coordinate.
 * @param width Width of the grid map.
 * @param height Height of the grid map.
 * @param collision_radius Collision radius considering robot radius + obstacle radius + security margin.
 */
AStar::AStar(const std::vector<Coord>& obstacles,
             Coord start,
             Coord goal,
             int width,
             int height,
             float collision_radius)
: obstacles_(obstacles),
  start_(start),
  goal_(goal),
  width_(width),
  height_(height),
  collision_radius_(collision_radius)
{
    AStarNode n0;
    n0.state = start_;
    n0.g = 0.0f;
    n0.h = heuristic(start_);
    n0.f = n0.g + n0.h;
    n0.path.push_back(start_);

    open_list_.push(n0);
}

/**
 * @brief Computes the heuristic cost to the goal.
 *
 * Uses the Euclidean distance between the current state and the goal.
 *
 * @param s Current grid coordinate.
 * @return Estimated cost to reach the goal.
 */
float AStar::heuristic(const Coord& s) const {
    return std::hypot(float(s.r - goal_.r),
                      float(s.c - goal_.c));
}

/**
 * @brief Checks whether a grid coordinate is in collision.
 *
 * A collision occurs if the Euclidean distance between the state
 * and any obstacle is smaller than or equal to the robot radius.
 *
 * @param s Grid coordinate to be checked.
 * @return True if the state is in collision, false otherwise.
 */
bool AStar::collides(const Coord& s) const {
    for (const auto& obs : obstacles_) {
        float d = std::hypot(float(s.r - obs.r),
                             float(s.c - obs.c));
        if (d <= collision_radius_)
            return true;
    }
    return false;
}

/**
 * @brief Generates valid neighboring nodes from the current node.
 *
 * Neighbors are generated using an 8-connected grid motion model.
 * Boundary constraints and collision checking are enforced.
 *
 * @param current Current A* node being expanded.
 * @return Vector containing all valid neighboring nodes.
 */
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

        // Grid boundary check
        if (next.r < 0 || next.r >= height_ ||
            next.c < 0 || next.c >= width_)
            continue;

        // Circular collision check
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

/**
 * @brief Indicates whether the planner has determined that no solution exists.
 *
 * @return True if no solution has been found, false otherwise.
 */
bool AStar::no_solution() const {
    return no_solution_;
}

/**
 * @brief Executes one iteration of the A* search algorithm.
 *
 * Expands the node with the lowest total cost from the open list.
 * The search is performed incrementally, allowing step-by-step execution.
 *
 * @return The grid coordinate expanded during this iteration.
 *         Returns {-1, -1} if no solution exists.
 */
Coord AStar::step() {
    if (open_list_.empty()) {
        no_solution_ = true;
        return {-1, -1}; // Invalid state
    }

    AStarNode current = open_list_.top();
    open_list_.pop();

    // Goal check
    if (current.state == goal_) {
        goal_reached_ = true;
        solution_path_ = current.path;
        return current.state;
    }

    // Closed list check
    auto it = closed_list_.find(current.state);
    if (it != closed_list_.end() && it->second <= current.g)
        return current.state;

    closed_list_[current.state] = current.g;

    // Expand neighbors
    for (auto& n : generate_neighbors(current))
        open_list_.push(n);

    return current.state;
}

/**
 * @brief Indicates whether the goal has been reached.
 *
 * @return True if the goal has been reached, false otherwise.
 */
bool AStar::goal_reached() const {
    return goal_reached_;
}

/**
 * @brief Returns the solution path once the goal is reached.
 *
 * @return Vector of grid coordinates representing the solution path.
 */
std::vector<Coord> AStar::path() const {
    return solution_path_;
}
