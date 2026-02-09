/**
 * @file AStar.hpp
 * @brief Grid-based A* global path planning algorithm.
 *
 * This header defines the data structures and interface for a classical
 * A* search algorithm applied to a 2D grid environment with circular
 * obstacles.
 */

#pragma once

#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "Coord.hpp"

/**
 * @struct AStarNode
 * @brief Node representation used by the A* search.
 *
 * Each node stores the current state, cost-to-come (g),
 * heuristic cost-to-go (h), total cost (f), and the path
 * followed to reach this state.
 */
struct AStarNode {
    /// Grid state (row, column)
    Coord state;

    /// Cost from the start node to this node
    float g;

    /// Heuristic estimate from this node to the goal
    float h;

    /// Total estimated cost (f = g + h)
    float f;

    /// Discrete path from the start to this node
    std::vector<Coord> path;
};

/**
 * @struct CompareAStarNode
 * @brief Comparison functor for priority queue ordering.
 *
 * Nodes with lower total cost (f) have higher priority.
 */
struct CompareAStarNode {
    bool operator()(const AStarNode& a,
                    const AStarNode& b) const {
        return a.f > b.f;
    }
};

/**
 * @class AStar
 * @brief Global path planner based on the A* search algorithm.
 *
 * This class implements a grid-based A* planner that computes a
 * collision-free path between a start and a goal position in a
 * discretized 2D environment. Obstacles are modeled as circular
 * regions, and collision checking is performed accordingly.
 */
class AStar {
public:
    /**
     * @brief Constructs an A* planner instance.
     *
     * @param obstacles List of obstacle positions.
     * @param start Initial robot position.
     * @param goal Target position.
     * @param width Width of the grid environment.
     * @param height Height of the grid environment.
     * @param collision_radius Collision radius considering robot radius + obstacle radius + security margin.
     */
    AStar(const std::vector<Coord>& obstacles,
          Coord start,
          Coord goal,
          int width,
          int height,
          float collision_radius);

    /**
     * @brief Executes a single iteration of the A* algorithm.
     *
     * Expands the lowest-cost node in the open list and updates
     * the search frontier.
     *
     * @return Current state expanded by the algorithm.
     */
    Coord step();

    /**
     * @brief Indicates whether the goal state has been reached.
     *
     * @return True if a valid path to the goal was found.
     */
    bool goal_reached() const;

    /**
     * @brief Returns the computed solution path.
     *
     * The path is only valid if the goal has been reached.
     *
     * @return Sequence of grid states from start to goal.
     */
    std::vector<Coord> path() const;

    /**
     * @brief Indicates that no feasible path exists.
     *
     * @return True if the open list was exhausted without
     *         reaching the goal.
     */
    bool no_solution() const;

private:
    /**
     * @brief Computes the heuristic cost-to-go.
     *
     * Uses the Euclidean distance between the current state
     * and the goal state.
     *
     * @param s Current grid state.
     * @return Heuristic estimate to the goal.
     */
    float heuristic(const Coord& s) const;

    /**
     * @brief Checks for collision at a given grid state.
     *
     * @param s Grid state to be tested.
     * @return True if the state is in collision with an obstacle.
     */
    bool collides(const Coord& s) const;

    /**
     * @brief Generates valid neighboring nodes.
     *
     * Considers 8-connected grid motion and performs
     * boundary and collision checks.
     *
     * @param current Current node being expanded.
     * @return List of valid neighboring nodes.
     */
    std::vector<AStarNode> generate_neighbors(const AStarNode& current);

    /// Flag indicating that no solution exists
    bool no_solution_ = false;

    /// Priority queue of nodes to be expanded
    std::priority_queue<
        AStarNode,
        std::vector<AStarNode>,
        CompareAStarNode
    > open_list_;

    /// Closed set storing the lowest cost for each visited state
    std::unordered_map<Coord, float, CoordHash> closed_list_;

    /// List of obstacle positions
    std::vector<Coord> obstacles_;

    /// Solution path from start to goal
    std::vector<Coord> solution_path_;

    /// Start and goal states
    Coord start_, goal_;

    /// Grid dimensions
    int width_, height_;

    /// Robot collision radius
    float collision_radius_;

    /// Flag indicating successful termination
    bool goal_reached_ = false;
};
