#include "grid_path_planner/MapParser.hpp"

#include <fstream>
#include <stdexcept>

namespace grid_path_planner {
namespace {

bool is_supported_cell(char cell)
{
    return cell == '.' || cell == '#' || cell == 'S' || cell == 'G';
}

void remove_trailing_carriage_return(std::string& line)
{
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
}

} // namespace

MapDefinition parse_map_lines(const std::vector<std::string>& lines)
{
    std::vector<std::string> rows;
    rows.reserve(lines.size());

    for (auto line : lines) {
        remove_trailing_carriage_return(line);
        if (!line.empty()) {
            rows.push_back(line);
        }
    }

    if (rows.empty()) {
        throw GridError("map file does not contain any grid rows");
    }

    MapDefinition map;
    map.display_rows = rows;

    for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
        for (int col = 0; col < static_cast<int>(rows[static_cast<std::size_t>(row)].size()); ++col) {
            const char cell = rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            if (!is_supported_cell(cell)) {
                throw GridError("map contains unsupported cell character");
            }

            if (cell == 'S') {
                if (map.start.has_value()) {
                    throw GridError("map contains more than one start cell");
                }
                map.start = Point{row, col};
            } else if (cell == 'G') {
                if (map.goal.has_value()) {
                    throw GridError("map contains more than one goal cell");
                }
                map.goal = Point{row, col};
            }
        }
    }

    map.grid = Grid::from_text_rows(rows);
    return map;
}

MapDefinition load_map_file(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open map file: " + path);
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }

    return parse_map_lines(lines);
}

std::vector<std::string> render_path(const MapDefinition& map, const std::vector<Point>& path)
{
    auto rows = map.display_rows;

    for (const auto& point : path) {
        char& cell = rows[static_cast<std::size_t>(point.row)][static_cast<std::size_t>(point.col)];
        if (cell == '.') {
            cell = '*';
        }
    }

    return rows;
}

} // namespace grid_path_planner
