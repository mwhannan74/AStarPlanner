cmake_minimum_required(VERSION 3.16)
project(AStarPlanner LANGUAGES CXX)

include(CTest)

# ---------- C++ settings -----------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------- Local dependency paths -------------------------------------------
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/local_paths.cmake")

# ---------- Library target ---------------------------------------------------
add_library(astar_planner INTERFACE)
add_library(AStarPlanner::astar_planner ALIAS astar_planner)

target_include_directories(astar_planner
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ASTAR_PLANNER_EIGEN3_INCLUDE_DIR}"
)

# ---------- Demo executable --------------------------------------------------
option(ASTAR_PLANNER_BUILD_DEMO "Build the demo executables." ON)
option(ASTAR_PLANNER_ENABLE_VISUALIZATION "Enable visualization support via MatPlotOpenCV." ON)

if(ASTAR_PLANNER_ENABLE_VISUALIZATION)
    add_subdirectory(
        "${ASTAR_PLANNER_MPOCV_SOURCE_DIR}"
        "${CMAKE_BINARY_DIR}/external/mpocv"
    )

    add_library(astar_planner_visualization INTERFACE)
    add_library(AStarPlanner::astar_planner_visualization ALIAS astar_planner_visualization)

    target_link_libraries(astar_planner_visualization
        INTERFACE
            AStarPlanner::astar_planner
            mpocv
    )
endif()

if(ASTAR_PLANNER_BUILD_DEMO)
    if(NOT ASTAR_PLANNER_ENABLE_VISUALIZATION)
        message(FATAL_ERROR "ASTAR_PLANNER_BUILD_DEMO requires ASTAR_PLANNER_ENABLE_VISUALIZATION to be ON.")
    endif()

    add_executable(a_star_planner_demo
        demo/a_star_planner_demo.cpp
    )

    add_executable(operation_area_demo
        demo/operation_area_demo.cpp
    )

    target_link_libraries(a_star_planner_demo
        PRIVATE
            AStarPlanner::astar_planner_visualization
    )

    target_link_libraries(operation_area_demo
        PRIVATE
            AStarPlanner::astar_planner_visualization
    )
endif()

# ---------- Tests -------------------------------------------------------------
if(BUILD_TESTING)
    add_executable(a_star_planner_tests
        tests/a_star_planner_tests.cpp
    )

    target_link_libraries(a_star_planner_tests
        PRIVATE
            AStarPlanner::astar_planner
    )

    add_test(
        NAME a_star_planner_tests
        COMMAND a_star_planner_tests
    )
endif()
