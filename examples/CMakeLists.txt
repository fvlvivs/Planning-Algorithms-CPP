add_executable(dijkstra_2D dijkstra_2D.cpp
        PlotUtils.hpp)


#set(Matplotplusplus_DIR "../matplotplusplus")
#include_directories(${Matplotplusplus_DIR})
#
target_link_libraries(dijkstra_2D PUBLIC matplot)
target_link_libraries(dijkstra_2D PRIVATE XiPlanningAlgorithms)
