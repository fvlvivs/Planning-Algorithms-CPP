add_library(
        XiPlanningAlgorithms
        INTERFACE
        XiPlanningAlgorithms/DiscreteSearch.hpp
)

set(EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

find_package(Eigen3 REQUIRED)

target_link_libraries(
        XiPlanningAlgorithms
        INTERFACE
        Eigen3::Eigen
)

target_include_directories(
        XiPlanningAlgorithms
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)