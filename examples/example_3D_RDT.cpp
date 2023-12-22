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
#include <XiPlanningAlgorithms/Core.hpp>
#include <XiPlanningAlgorithms/Polyhedra.hpp>
#include <XiPlanningAlgorithms/RDT.hpp>

#include "PlotUtils.hpp"


int main() {
    printf("--- 3D RDT example ---\n");

    Node3d* start = new Node3d({0, 0, 0});
    Node3d* goal = new Node3d({6, 8, 3});
    RDT<double, 3> rdt(start, goal);

    // set system
    Polyhedra<double> system;
    createParallelepiped<double>(system, 0.5, 0.5, 0.5);
    rdt.setSystem(system);

    // add obstacles
    Polyhedra<double> obstacle;
    createParallelepiped<double>(obstacle, 2, 2, 0.5);
    obstacle.moveToPoint({2, 2, 0});

    Polyhedra<double> obstacle2;
    createParallelepiped<double>(obstacle2, 0.8, 0.8, 4);
    obstacle2.moveToPoint({4, 6, 2});

    rdt.addObstacle(obstacle);
    rdt.addObstacle(obstacle2);

    // set distribution
    std::vector<std::pair<double, double>> limits;
    limits.emplace_back(-1, 10);
    limits.emplace_back(-1, 10);
    limits.emplace_back(-1, 5);
    PointDistribution3d distribution(limits);
    rdt.setDistribution(distribution);

    rdt.setMaximumIterations(3000);
    rdt.setNeighbourRadius(1.2);
    rdt.setGoalRangeRadius(0.5);
    rdt.setRewiringRadius(0.3);
    rdt.setPathCheckResolution(0.025);

    rdt.run();
    bool success = rdt.isSolutionFound();

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

    std::vector<Node3d*> nodes;
    rdt.getNodes(nodes);
    plotRDTNodes(nodes, "b");

    std::vector<Node3d*> solution;
    rdt.getSolution(solution);
    plotPath(solution, "r");

    matplot::xlim({limits[0].first, limits[0].second});
    matplot::ylim({limits[1].first, limits[1].second});
    matplot::zlim({limits[2].first, limits[2].second});
    matplot::xlabel("x [m]");
    matplot::ylabel("y [m]");
    matplot::zlabel("z [m]");
    matplot::title("3D RDT");
    matplot::show();

    return 0;
}