#include "grid_path_planner/AStarPlanner.hpp"
#include "grid_path_planner/MapParser.hpp"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct CliOptions {
    std::string map_path;
    grid_path_planner::Algorithm algorithm = grid_path_planner::Algorithm::AStar;
    bool allow_diagonal = false;
    bool show_help = false;
    std::optional<grid_path_planner::Point> start;
    std::optional<grid_path_planner::Point> goal;
};

void print_usage(std::ostream& output)
{
    output
        << "cpp-grid-path-planner --map <file> [--algorithm astar|dijkstra] [--diagonal]\n"
        << "                      [--start row,col --goal row,col]\n\n"
        << "Options:\n"
        << "  --map <file>               Path to a text occupancy grid.\n"
        << "  --algorithm <name>         astar (default) or dijkstra.\n"
        << "  --diagonal                 Allow diagonal movement.\n"
        << "  --start <row,col>          Override S from the map.\n"
        << "  --goal <row,col>           Override G from the map.\n"
        << "  --help                     Show this help text.\n";
}

std::string require_value(const std::vector<std::string>& args, std::size_t& index, const std::string& option)
{
    if (index + 1 >= args.size()) {
        throw std::invalid_argument(option + " requires a value");
    }
    ++index;
    return args[index];
}

grid_path_planner::Point parse_point(const std::string& value)
{
    std::istringstream stream(value);
    int row = 0;
    int col = 0;
    char comma = '\0';

    if (!(stream >> row >> comma >> col) || comma != ',') {
        throw std::invalid_argument("point must use row,col format");
    }

    return grid_path_planner::Point{row, col};
}

CliOptions parse_args(int argc, char** argv)
{
    const std::vector<std::string> args(argv + 1, argv + argc);
    CliOptions options;

    for (std::size_t index = 0; index < args.size(); ++index) {
        const auto& arg = args[index];

        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
        } else if (arg == "--map") {
            options.map_path = require_value(args, index, arg);
        } else if (arg == "--algorithm") {
            const auto value = require_value(args, index, arg);
            if (value == "astar") {
                options.algorithm = grid_path_planner::Algorithm::AStar;
            } else if (value == "dijkstra") {
                options.algorithm = grid_path_planner::Algorithm::Dijkstra;
            } else {
                throw std::invalid_argument("algorithm must be astar or dijkstra");
            }
        } else if (arg == "--diagonal") {
            options.allow_diagonal = true;
        } else if (arg == "--start") {
            options.start = parse_point(require_value(args, index, arg));
        } else if (arg == "--goal") {
            options.goal = parse_point(require_value(args, index, arg));
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    return options;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const auto cli = parse_args(argc, argv);
        if (cli.show_help) {
            print_usage(std::cout);
            return EXIT_SUCCESS;
        }

        if (cli.map_path.empty()) {
            print_usage(std::cerr);
            return EXIT_FAILURE;
        }

        auto map = grid_path_planner::load_map_file(cli.map_path);
        const auto start = cli.start.value_or(map.start.value_or(grid_path_planner::Point{-1, -1}));
        const auto goal = cli.goal.value_or(map.goal.value_or(grid_path_planner::Point{-1, -1}));

        if (start.row < 0 || goal.row < 0) {
            throw std::invalid_argument("start and goal must be supplied either in the map or through CLI flags");
        }

        grid_path_planner::PlannerOptions options;
        options.algorithm = cli.algorithm;
        options.allow_diagonal = cli.allow_diagonal;

        const grid_path_planner::Planner planner(map.grid);
        const auto result = planner.plan(start, goal, options);

        if (!result.found()) {
            std::cout << "No path found.\n";
            std::cout << "Nodes expanded: " << result.nodes_expanded << '\n';
            return EXIT_FAILURE;
        }

        for (const auto& row : grid_path_planner::render_path(map, result.path)) {
            std::cout << row << '\n';
        }

        std::cout << "\nPath length: " << result.path.size() << " nodes\n";
        std::cout << "Path cost: " << std::fixed << std::setprecision(3) << result.cost << '\n';
        std::cout << "Nodes expanded: " << result.nodes_expanded << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
