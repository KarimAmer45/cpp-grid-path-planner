#include "grid_path_planner/Grid.hpp"

#include <array>

namespace grid_path_planner {

Grid::Grid(int rows, int cols, bool initially_blocked)
    : rows_(rows),
      cols_(cols)
{
    if (rows < 0 || cols < 0) {
        throw GridError("grid dimensions must be non-negative");
    }

    blocked_.assign(static_cast<std::size_t>(rows * cols), initially_blocked);
}

Grid Grid::from_text_rows(const std::vector<std::string>& rows)
{
    if (rows.empty()) {
        throw GridError("map must contain at least one row");
    }

    const auto cols = static_cast<int>(rows.front().size());
    if (cols == 0) {
        throw GridError("map rows must not be empty");
    }

    Grid grid(static_cast<int>(rows.size()), cols);
    for (int row = 0; row < grid.rows(); ++row) {
        if (static_cast<int>(rows[static_cast<std::size_t>(row)].size()) != cols) {
            throw GridError("map rows must all have the same width");
        }

        for (int col = 0; col < cols; ++col) {
            const char value = rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            if (value == '#') {
                grid.set_blocked(Point{row, col}, true);
            }
        }
    }

    return grid;
}

int Grid::rows() const noexcept
{
    return rows_;
}

int Grid::cols() const noexcept
{
    return cols_;
}

bool Grid::empty() const noexcept
{
    return rows_ == 0 || cols_ == 0;
}

bool Grid::in_bounds(Point point) const noexcept
{
    return point.row >= 0 && point.row < rows_ && point.col >= 0 && point.col < cols_;
}

bool Grid::is_blocked(Point point) const
{
    if (!in_bounds(point)) {
        throw GridError("point is outside grid bounds");
    }
    return blocked_[index(point)];
}

bool Grid::is_free(Point point) const
{
    return in_bounds(point) && !blocked_[index(point)];
}

void Grid::set_blocked(Point point, bool blocked)
{
    if (!in_bounds(point)) {
        throw GridError("point is outside grid bounds");
    }
    blocked_[index(point)] = blocked;
}

std::vector<Point> Grid::neighbors(Point point, bool allow_diagonal) const
{
    static constexpr std::array<Point, 4> cardinal{{
        Point{-1, 0},
        Point{0, 1},
        Point{1, 0},
        Point{0, -1},
    }};

    static constexpr std::array<Point, 4> diagonal{{
        Point{-1, -1},
        Point{-1, 1},
        Point{1, 1},
        Point{1, -1},
    }};

    std::vector<Point> result;
    result.reserve(allow_diagonal ? 8U : 4U);

    for (const auto& delta : cardinal) {
        const Point candidate{point.row + delta.row, point.col + delta.col};
        if (is_free(candidate)) {
            result.push_back(candidate);
        }
    }

    if (!allow_diagonal) {
        return result;
    }

    for (const auto& delta : diagonal) {
        const Point candidate{point.row + delta.row, point.col + delta.col};
        const Point row_step{point.row + delta.row, point.col};
        const Point col_step{point.row, point.col + delta.col};
        if (is_free(candidate) && is_free(row_step) && is_free(col_step)) {
            result.push_back(candidate);
        }
    }

    return result;
}

std::size_t Grid::index(Point point) const
{
    return static_cast<std::size_t>(point.row * cols_ + point.col);
}

} // namespace grid_path_planner
