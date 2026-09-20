set(
    ASTAR_PLANNER_EIGEN3_INCLUDE_DIR
    "C:/eigen"
    CACHE PATH
    "Path to the Eigen include directory."
)

set(
    ASTAR_PLANNER_MPOCV_SOURCE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../MatPlotOpenCV"
    CACHE PATH
    "Path to the MatPlotOpenCV source directory."
)

message(STATUS "AStarPlanner Eigen include dir: ${ASTAR_PLANNER_EIGEN3_INCLUDE_DIR}")
message(STATUS "AStarPlanner MatPlotOpenCV source dir: ${ASTAR_PLANNER_MPOCV_SOURCE_DIR}")
