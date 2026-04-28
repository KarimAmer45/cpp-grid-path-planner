#include "grid_path_planner/AStarPlanner.hpp"
#include "grid_path_planner/MapParser.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void parses_start_goal_and_obstacles()
{
    const auto map = grid_path_planner::parse_map_lines({
        "S.#",
        "..G",
    });

    require(map.grid.rows() == 2, "expected two rows");
    require(map.grid.cols() == 3, "expected three columns");
    require(map.start == grid_path_planner::Point{0, 0}, "expected start at 0,0");
    require(map.goal == grid_path_planner::Point{1, 2}, "expected goal at 1,2");
    require(map.grid.is_blocked(grid_path_planner::Point{0, 2}), "expected obstacle at 0,2");
}

void astar_finds_cardinal_path()
{
    const auto map = grid_path_planner::parse_map_lines({
        "S..",
        "##.",
        "..G",
    });

    const grid_path_planner::Planner planner(map.grid);
    const auto result = planner.plan(*map.start, *map.goal);

    require(result.found(), "expected a path");
    require(result.path.front() == *map.start, "path should start at S");
    require(result.path.back() == *map.goal, "path should end at G");
    require(result.path.size() == 5, "expected five path nodes");
    require(std::abs(result.cost - 4.0) < 0.0001, "expected cardinal path cost of 4");
}

void dijkstra_matches_astar_cost()
{
    const auto map = grid_path_planner::parse_map_lines({
        "S....",
        ".###.",
        "....G",
    });

    const grid_path_planner::Planner planner(map.grid);

    grid_path_planner::PlannerOptions dijkstra;
    dijkstra.algorithm = grid_path_planner::Algorithm::Dijkstra;

    const auto astar_result = planner.plan(*map.start, *map.goal);
    const auto dijkstra_result = planner.plan(*map.start, *map.goal, dijkstra);

    require(astar_result.found(), "expected A* path");
    require(dijkstra_result.found(), "expected Dijkstra path");
    require(std::abs(astar_result.cost - dijkstra_result.cost) < 0.0001, "expected matching costs");
}

void reports_no_path()
{
    const auto map = grid_path_planner::parse_map_lines({
        "S#G",
    });

    const grid_path_planner::Planner planner(map.grid);
    const auto result = planner.plan(*map.start, *map.goal);

    require(!result.found(), "expected no path");
}

void diagonal_path_uses_lower_cost()
{
    const auto map = grid_path_planner::parse_map_lines({
        "S..",
        "...",
        "..G",
    });

    const grid_path_planner::Planner planner(map.grid);

    grid_path_planner::PlannerOptions options;
    options.allow_diagonal = true;

    const auto result = planner.plan(*map.start, *map.goal, options);

    require(result.found(), "expected diagonal path");
    require(result.path.size() == 3, "expected direct diagonal route");
    require(std::abs(result.cost - (2.0 * std::sqrt(2.0))) < 0.0001, "expected two diagonal moves");
}

} // namespace

int main()
{
    try {
        parses_start_goal_and_obstacles();
        astar_finds_cardinal_path();
        dijkstra_matches_astar_cost();
        reports_no_path();
        diagonal_path_uses_lower_cost();
        std::cout << "planner_tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "planner_tests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
