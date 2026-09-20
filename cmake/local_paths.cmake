set(
    VISGRAPH_EIGEN3_INCLUDE_DIR
    "C:/eigen"
    CACHE PATH
    "Path to the Eigen include directory."
)

set(
    VISGRAPH_MPOCV_SOURCE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../MatPlotOpenCV"
    CACHE PATH
    "Path to the MatPlotOpenCV source directory."
)

message(STATUS "VisGraphPlanner Eigen include dir: ${VISGRAPH_EIGEN3_INCLUDE_DIR}")
message(STATUS "VisGraphPlanner MatPlotOpenCV source dir: ${VISGRAPH_MPOCV_SOURCE_DIR}")
