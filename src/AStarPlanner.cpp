#include "grid_path_planner/AStarPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace grid_path_planner {
namespace {

struct QueueNode {
    Point point;
    double priority = 0.0;

    friend bool operator>(const QueueNode& lhs, const QueueNode& rhs) noexcept
    {
        return lhs.priority > rhs.priority;
    }
};

double movement_cost(Point from, Point to)
{
    const int d_row = std::abs(from.row - to.row);
    const int d_col = std::abs(from.col - to.col);
    return d_row == 1 && d_col == 1 ? std::sqrt(2.0) : 1.0;
}

double heuristic(Point from, Point to, bool allow_diagonal)
{
    const auto d_row = static_cast<double>(std::abs(from.row - to.row));
    const auto d_col = static_cast<double>(std::abs(from.col - to.col));

    if (!allow_diagonal) {
        return d_row + d_col;
    }

    const double diagonal = std::min(d_row, d_col);
    const double straight = std::max(d_row, d_col) - diagonal;
    return diagonal * std::sqrt(2.0) + straight;
}

std::vector<Point> reconstruct_path(
    const std::unordered_map<Point, Point>& came_from,
    Point start,
    Point goal)
{
    std::vector<Point> path;
    Point current = goal;
    path.push_back(current);

    while (current != start) {
        const auto next = came_from.find(current);
        if (next == came_from.end()) {
            return {};
        }
        current = next->second;
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

Planner::Planner(const Grid& grid)
    : grid_(grid)
{
}

PlanResult Planner::plan(Point start, Point goal, PlannerOptions options) const
{
    if (!grid_.is_free(start)) {
        throw std::invalid_argument("start must be inside the grid and not blocked");
    }

    if (!grid_.is_free(goal)) {
        throw std::invalid_argument("goal must be inside the grid and not blocked");
    }

    PlanResult result;
    if (start == goal) {
        result.path = {start};
        return result;
    }

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> frontier;
    std::unordered_map<Point, Point> came_from;
    std::unordered_map<Point, double> cost_so_far;

    frontier.push(QueueNode{start, 0.0});
    cost_so_far[start] = 0.0;

    while (!frontier.empty()) {
        const Point current = frontier.top().point;
        frontier.pop();
        ++result.nodes_expanded;

        if (current == goal) {
            result.path = reconstruct_path(came_from, start, goal);
            result.cost = cost_so_far[goal];
            return result;
        }

        for (const auto& next : grid_.neighbors(current, options.allow_diagonal)) {
            const double new_cost = cost_so_far[current] + movement_cost(current, next);
            const auto known = cost_so_far.find(next);

            if (known == cost_so_far.end() || new_cost < known->second) {
                cost_so_far[next] = new_cost;
                const double h = options.algorithm == Algorithm::AStar
                    ? heuristic(next, goal, options.allow_diagonal)
                    : 0.0;
                frontier.push(QueueNode{next, new_cost + h});
                came_from[next] = current;
            }
        }
    }

    result.cost = std::numeric_limits<double>::infinity();
    return result;
}

} // namespace grid_path_planner
