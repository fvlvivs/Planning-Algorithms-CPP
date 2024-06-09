// MIT License
//
// Copyright (c) 2023 Fulvio Di Luzio (fulvio.diluzio@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <cstdio>
#include <chrono>
#include <XiPlanningAlgorithms/Core.hpp>
#include <XiPlanningAlgorithms/Polyhedra.hpp>
#include <XiPlanningAlgorithms/AStar.hpp>

#include "PlotUtils.hpp"


int main() {
    printf("--- 3D A* example ---\n");

    Node3d* start = new Node3d({1, 1, 1});
    Node3d* goal = new Node3d({6, 7, 3});
    AStar<double, 3> astar(start, goal);

    // set system
    Polyhedra<double> system;
    createParallelepiped<double>(system, 0.5, 0.5, 0.5);
    astar.setSystem(system);

    // add obstacles
    Polyhedra<double> obstacle;
    createParallelepiped<double>(obstacle, 2, 2, 0.5);
    obstacle.moveToPoint({2, 2, 0});

    Polyhedra<double> obstacle2;
    createParallelepiped<double>(obstacle2, 0.8, 0.8, 4);
    obstacle2.moveToPoint({4, 6, 2});

    astar.addObstacle(obstacle);
    astar.addObstacle(obstacle2);

    // set space limits
    std::vector<std::pair<double, double>> limits;
    limits.emplace_back(0, 10);
    limits.emplace_back(0, 10);
    limits.emplace_back(0, 5);
    astar.setSpaceLimits(limits);

    // settings
    astar.setMaximumIterations(3000);

    const auto start_time = std::chrono::high_resolution_clock::now();
    astar.run();
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    printf("Planner took: %ld ms\n", duration);

    bool success = astar.isSolutionFound();
    if (!success) {
        printf("No solution found\n");
        return 0;
    }

    // plot
    auto fig = matplot::figure(true);
    fig->width(800);
    fig->height(800);
    matplot::hold(matplot::on);
    matplot::grid(matplot::on);

    system.moveToPoint(*start);
    plotPolyhedra<double>(system, "b");

    system.moveToPoint(*goal);
    plotPolyhedra<double>(system, "g");

    plotPolyhedra<double>(obstacle, "k");
    plotPolyhedra<double>(obstacle2, "k");

    std::vector<Node<double, 3> *> solution;
    astar.getSolution(solution);
    plotPath<double, 3>(solution, "r");

    matplot::xlim({limits[0].first, limits[0].second});
    matplot::ylim({limits[1].first, limits[1].second});
    matplot::zlim({limits[2].first, limits[2].second});
    matplot::title("3D A*");
    matplot::xlabel("x [m]");
    matplot::ylabel("y [m]");
    matplot::zlabel("z [m]");
    matplot::show();

    return 0;
}

