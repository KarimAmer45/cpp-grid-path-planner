#pragma once

#include "grid_path_planner/Point.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace grid_path_planner {

class Grid {
public:
    Grid() = default;
    Grid(int rows, int cols, bool initially_blocked = false);

    static Grid from_text_rows(const std::vector<std::string>& rows);

    [[nodiscard]] int rows() const noexcept;
    [[nodiscard]] int cols() const noexcept;
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] bool in_bounds(Point point) const noexcept;
    [[nodiscard]] bool is_blocked(Point point) const;
    [[nodiscard]] bool is_free(Point point) const;

    void set_blocked(Point point, bool blocked);
    [[nodiscard]] std::vector<Point> neighbors(Point point, bool allow_diagonal) const;

private:
    int rows_ = 0;
    int cols_ = 0;
    std::vector<bool> blocked_;

    [[nodiscard]] std::size_t index(Point point) const;
};

class GridError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

} // namespace grid_path_planner
