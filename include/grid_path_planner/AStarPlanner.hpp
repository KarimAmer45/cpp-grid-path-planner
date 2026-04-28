#pragma once

#include "grid_path_planner/Grid.hpp"
#include "grid_path_planner/Point.hpp"

#include <cstddef>
#include <vector>

namespace grid_path_planner {

enum class Algorithm {
    AStar,
    Dijkstra
};

struct PlannerOptions {
    Algorithm algorithm = Algorithm::AStar;
    bool allow_diagonal = false;
};

struct PlanResult {
    std::vector<Point> path;
    double cost = 0.0;
    std::size_t nodes_expanded = 0;

    [[nodiscard]] bool found() const noexcept
    {
        return !path.empty();
    }
};

class Planner {
public:
    explicit Planner(const Grid& grid);

    [[nodiscard]] PlanResult plan(Point start, Point goal, PlannerOptions options = {}) const;

private:
    const Grid& grid_;
};

} // namespace grid_path_planner
