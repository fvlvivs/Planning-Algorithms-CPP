add_library(
        PlanningAlgorithmsCPP
        INTERFACE
        PlanningAlgorithmsCPP/DiscreteSearch.hpp
)

set(EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

find_package(Eigen3 REQUIRED)

target_link_libraries(
        PlanningAlgorithmsCPP
        INTERFACE
        Eigen3::Eigen
)

target_include_directories(
        PlanningAlgorithmsCPP
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)