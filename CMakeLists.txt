cmake_minimum_required(VERSION 3.22)
project(planning-algorithms-cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(source)
add_subdirectory(examples)

set(Matplotplusplus_DIR "matplotplusplus")
include_directories(${Matplotplusplus_DIR})
add_subdirectory(${Matplotplusplus_DIR})