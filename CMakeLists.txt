cmake_minimum_required(VERSION 3.22)
project(xi-planning-algorithms)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(xi-planning-algorithms
        src/main.cpp
)


target_link_libraries(xi-planning-algorithms PUBLIC Eigen3::Eigen)

target_include_directories(xi-planning-algorithms PRIVATE include)
