#cmake_minimum_required(VERSION 3.22)
#project(xi-planning-algorithms)
#
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#
#find_package(Eigen3 REQUIRED)
#
#add_executable(xi-planning-algorithms
#    src/main.cpp
#)
#
#add_subdirectory(examples)
#
#set(Matplotplusplus_DIR "matplotplusplus")
#include_directories(${Matplotplusplus_DIR})
#add_subdirectory(${Matplotplusplus_DIR})
#
#target_link_libraries(xi-planning-algorithms PUBLIC Eigen3::Eigen)
#target_link_libraries(xi-planning-algorithms PUBLIC matplot)
#
#target_include_directories(xi-planning-algorithms PRIVATE source)
##target_include_directories(example_dijkstra_2D PRIVATE source)


cmake_minimum_required(VERSION 3.22)
project(xi-planning-algorithms)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(xi-planning-algorithms
    src/main.cpp
)

add_subdirectory(source)
add_subdirectory(examples)

set(Matplotplusplus_DIR "matplotplusplus")
include_directories(${Matplotplusplus_DIR})
add_subdirectory(${Matplotplusplus_DIR})

target_link_libraries(xi-planning-algorithms PUBLIC matplot)
target_link_libraries(xi-planning-algorithms PUBLIC XiPlanningAlgorithms)

#target_include_directories(xi-planning-algorithms PRIVATE source)
#target_include_directories(example_dijkstra_2D PRIVATE source)
