cmake_minimum_required(VERSION 3.16)
project(VisGraphPlanner LANGUAGES CXX)

include(CTest)

# ---------- C++ settings -----------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------- Local dependency paths -------------------------------------------
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/local_paths.cmake")

# ---------- Library target ---------------------------------------------------
add_library(visgraph INTERFACE)
add_library(VisGraphPlanner::visgraph ALIAS visgraph)

target_include_directories(visgraph
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${VISGRAPH_EIGEN3_INCLUDE_DIR}"
)

# ---------- Demo executable --------------------------------------------------
option(VISGRAPH_BUILD_DEMO "Build the demo executables." ON)
option(VISGRAPH_ENABLE_VISUALIZATION "Enable visualization support via MatPlotOpenCV." ON)

if(VISGRAPH_ENABLE_VISUALIZATION)
    add_subdirectory(
        "${VISGRAPH_MPOCV_SOURCE_DIR}"
        "${CMAKE_BINARY_DIR}/external/mpocv"
    )

    add_library(visgraph_visualization INTERFACE)
    add_library(VisGraphPlanner::visgraph_visualization ALIAS visgraph_visualization)

    target_link_libraries(visgraph_visualization
        INTERFACE
            VisGraphPlanner::visgraph
            mpocv
    )
endif()

if(VISGRAPH_BUILD_DEMO)
    if(NOT VISGRAPH_ENABLE_VISUALIZATION)
        message(FATAL_ERROR "VISGRAPH_BUILD_DEMO requires VISGRAPH_ENABLE_VISUALIZATION to be ON.")
    endif()

    add_executable(main_demo
        demo/main_demo.cpp
    )

    add_executable(operation_area_demo
        demo/operation_area_demo.cpp
    )

    target_link_libraries(main_demo
        PRIVATE
            VisGraphPlanner::visgraph_visualization
    )

    target_link_libraries(operation_area_demo
        PRIVATE
            VisGraphPlanner::visgraph_visualization
    )
endif()

# ---------- Tests -------------------------------------------------------------
if(BUILD_TESTING)
    add_executable(visgraph_tests
        tests/visibility_graph_tests.cpp
    )

    target_link_libraries(visgraph_tests
        PRIVATE
            VisGraphPlanner::visgraph
    )

    add_test(
        NAME visgraph_tests
        COMMAND visgraph_tests
    )
endif()
