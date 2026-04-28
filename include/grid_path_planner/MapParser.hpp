#pragma once

#include "grid_path_planner/Grid.hpp"
#include "grid_path_planner/Point.hpp"

#include <optional>
#include <string>
#include <vector>

namespace grid_path_planner {

struct MapDefinition {
    Grid grid;
    std::optional<Point> start;
    std::optional<Point> goal;
    std::vector<std::string> display_rows;
};

MapDefinition parse_map_lines(const std::vector<std::string>& lines);
MapDefinition load_map_file(const std::string& path);

std::vector<std::string> render_path(const MapDefinition& map, const std::vector<Point>& path);

} // namespace grid_path_planner
