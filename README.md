# cpp-grid-path-planner

A dependency-light C++17 grid path planner with a reusable library, a command-line tool, example maps, and a small test suite.

The planner supports A* and Dijkstra search on rectangular occupancy grids. Maps are plain text, where `#` is blocked space, `.` is free space, `S` is the start, and `G` is the goal.

## Features

- Modern C++17 library API
- A* and Dijkstra search
- Optional diagonal movement
- Text-map parser with clear validation errors
- CLI that overlays the discovered path on the map
- CMake build with CTest tests

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run

```bash
./build/cpp-grid-path-planner --map examples/maps/warehouse.txt --algorithm astar
```

On Windows with a multi-config generator, the executable may be under `build/Debug/` or `build/Release/`.

## CLI

```text
cpp-grid-path-planner --map <file> [--algorithm astar|dijkstra] [--diagonal]
                      [--start row,col --goal row,col]
```

If `--start` and `--goal` are not supplied, the CLI uses `S` and `G` from the map file.

Example output:

```text
S*G

Path length: 3 nodes
Path cost: 2.000
Nodes expanded: 3
```

## Map Format

Every non-empty line is a grid row, and every row must have the same width.

```text
S..#....
.#.#.##.
.#...#G.
.#####..
........
```

Legend:

- `S`: start
- `G`: goal
- `#`: obstacle
- `.`: free cell

## Library Usage

```cpp
#include <grid_path_planner/AStarPlanner.hpp>
#include <grid_path_planner/Grid.hpp>

using grid_path_planner::Grid;
using grid_path_planner::Planner;
using grid_path_planner::PlannerOptions;
using grid_path_planner::Point;

Grid grid(5, 5);
Planner planner(grid);

PlannerOptions options;
options.allow_diagonal = true;

const auto result = planner.plan(Point{0, 0}, Point{4, 4}, options);
if (result.found()) {
    // result.path contains start through goal.
}
```

---

## Benchmarks

Compiled with `g++ -std=c++17 -O2`. Timed with 20 runs per grid size (median reported). Maps generated with 18% random obstacle density.

| Grid size | A* median (ms) | Dijkstra median (ms) | A* speedup |
|---|---|---|---|
| 10 × 10 | 1.076 | 1.140 | 1.06× |
| 50 × 50 | 1.192 | 1.104 | 0.93× (trivial path) |
| 200 × 200 | 1.630 | 1.555 | 0.95× (trivial path) |
| **500 × 500** | **15.57** | **175.96** | **11.3×** |

On complex maps where the heuristic is meaningful (500×500), A* expands **~30,600 nodes** versus Dijkstra's full frontier, achieving an **11.3× wall-clock speedup**.

Build and reproduce:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
# Run on a large map
./build/cpp-grid-path-planner --map examples/maps/warehouse.txt --algorithm astar
./build/cpp-grid-path-planner --map examples/maps/warehouse.txt --algorithm dijkstra
```
