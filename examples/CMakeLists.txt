add_executable(
        example_2D_dijkstra.cpp
        example_2D_dijkstra.cpp
        PlotUtils.hpp)

add_executable(
        example_3D_astar
        example_3D_astar.cpp
        PlotUtils.hpp
)

add_executable(
        example_2D_RDT
        example_2D_RDT.cpp
        PlotUtils.hpp
)

add_executable(
        example_3D_RDT
        example_3D_RDT.cpp
        PlotUtils.hpp
)

target_link_libraries(example_2D_dijkstra.cpp PUBLIC matplot)
target_link_libraries(example_2D_dijkstra.cpp PRIVATE PlanningAlgorithmsCPP)

target_link_libraries(example_3D_astar PUBLIC matplot)
target_link_libraries(example_3D_astar PRIVATE PlanningAlgorithmsCPP)

target_link_libraries(example_2D_RDT PUBLIC matplot)
target_link_libraries(example_2D_RDT PRIVATE PlanningAlgorithmsCPP)

target_link_libraries(example_3D_RDT PUBLIC matplot)
target_link_libraries(example_3D_RDT PRIVATE PlanningAlgorithmsCPP)