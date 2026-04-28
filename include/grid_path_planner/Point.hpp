#pragma once

#include <cstddef>
#include <functional>

namespace grid_path_planner {

struct Point {
    int row = 0;
    int col = 0;

    friend bool operator==(const Point& lhs, const Point& rhs) noexcept
    {
        return lhs.row == rhs.row && lhs.col == rhs.col;
    }

    friend bool operator!=(const Point& lhs, const Point& rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

} // namespace grid_path_planner

template <>
struct std::hash<grid_path_planner::Point> {
    std::size_t operator()(const grid_path_planner::Point& point) const noexcept
    {
        const auto row_hash = std::hash<int>{}(point.row);
        const auto col_hash = std::hash<int>{}(point.col);
        return row_hash ^ (col_hash + 0x9e3779b9U + (row_hash << 6U) + (row_hash >> 2U));
    }
};
