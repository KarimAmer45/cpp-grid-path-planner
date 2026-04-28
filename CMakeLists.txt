cmake_minimum_required(VERSION 3.16)

project(cpp_grid_path_planner
    VERSION 0.1.0
    DESCRIPTION "A C++17 grid-based path planning library and CLI"
    LANGUAGES CXX)

option(CPP_GRID_PATH_PLANNER_BUILD_TESTS "Build planner tests" ON)

add_library(grid_path_planner
    src/AStarPlanner.cpp
    src/Grid.cpp
    src/MapParser.cpp)

target_include_directories(grid_path_planner
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include)

target_compile_features(grid_path_planner PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(grid_path_planner PRIVATE /W4 /permissive-)
else()
    target_compile_options(grid_path_planner PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(cpp-grid-path-planner src/main.cpp)
target_link_libraries(cpp-grid-path-planner PRIVATE grid_path_planner)

if(CPP_GRID_PATH_PLANNER_BUILD_TESTS)
    enable_testing()

    add_executable(planner_tests tests/planner_tests.cpp)
    target_link_libraries(planner_tests PRIVATE grid_path_planner)
    add_test(NAME planner_tests COMMAND planner_tests)
endif()
