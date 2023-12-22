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
#include <XiPlanningAlgorithms/Polygon.hpp>
#include <XiPlanningAlgorithms/Dijkstra.hpp>

#include "PlotUtils.hpp"


int main() {
    printf("--- 2D Dijkstra example ---\n");

    Node2d* start = new Node2d({0, 0});
    Node2d* goal = new Node2d({6, 7});
    Dijkstra<double, 2> dijkstra(start, goal);

    // set system
    Point2d p1({0, 0});
    Point2d p2({0, 0.5});
    Point2d p3({0.5, 0.5});
    Point2d p4({0.5, 0});
    Polygon<double, 2> system({p1, p2, p3, p4});
    dijkstra.setSystem(system);

    // add obstacle
    Point2d o1({2, 2});
    Point2d o2({2, 6});
    Point2d o3({4, 2});
    Point2d o4({4, 6});
    Polygon<double, 2> obstacle({o1, o2, o3, o4});
    dijkstra.addObstacle(obstacle);

    // set space limits
    std::vector<std::pair<double, double>> limits;
    limits.emplace_back(-1, 10);
    limits.emplace_back(-1, 10);
    dijkstra.setSpaceLimits(limits);

    // run
    dijkstra.run();

    bool success = dijkstra.isSolutionFound();
    if (!success) {
        printf("No solution found\n");
        return 0;
    }

    // plot
    auto fig = matplot::figure(true);
    matplot::hold(matplot::on);
    matplot::grid(matplot::on);

    system.moveToPoint(*start);
    plotPolygon<double, 2>(system, "b");

    system.moveToPoint(*goal);
    plotPolygon<double, 2>(system, "g");

    plotPolygon<double, 2>(obstacle, "k");

    std::vector<Node<double, 2> *> solution;
    dijkstra.getSolution(solution);
    plotPath<double, 2>(solution, "r");

    matplot::xlim({limits[0].first, limits[0].second});
    matplot::ylim({limits[1].first, limits[1].second});
    matplot::title("2D Dijkstra");
    matplot::xlabel("x");
    matplot::ylabel("y");
    matplot::show();

    return 0;
}
