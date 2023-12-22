add_executable(
        example_dijkstra_2D
        example_2D_dijkstra.cpp
        PlotUtils.hpp)

add_executable(
        example_3D_astar
        example_3D_astar.cpp
        PlotUtils.hpp
)

target_link_libraries(dijkstra_2D PUBLIC matplot)
target_link_libraries(dijkstra_2D PRIVATE XiPlanningAlgorithms)

target_link_libraries(example_3D_astar PUBLIC matplot)
target_link_libraries(example_3D_astar PRIVATE XiPlanningAlgorithms)
