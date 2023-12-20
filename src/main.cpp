// // MIT License
// //
// // Copyright (c) 2023 Fulvio Di Luzio (fulvio.diluzio@gmail.com)
// //
// // Permission is hereby granted, free of charge, to any person obtaining a copy
// // of this software and associated documentation files (the "Software"), to deal
// // in the Software without restriction, including without limitation the rights
// // to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// // copies of the Software, and to permit persons to whom the Software is
// // furnished to do so, subject to the following conditions:
// //
// // The above copyright notice and this permission notice shall be included in all
// // copies or substantial portions of the Software.
// //
// // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// // AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// // OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// // SOFTWARE.
//
// #source "Geometry.hpp"
// #source "Polygon.hpp"
// #source "Polyhedra.hpp"
// #source "RDT.hpp"
// #source "SeparatingAxis.hpp"
// #source "Dijkstra.hpp"
// #source "AStar.hpp"
//
// #source <matplot/matplot.h>
// #source <thread>
//
//
// void plotPolygon(Polygon<float, 2>& poly, std::string color) {
//     std::vector<float> x;
//     std::vector<float> y;
//     std::vector<Point2f> vertices;
//     poly.getVertices(vertices);
//     for (auto vertex: vertices) {
//         x.push_back(vertex[0]);
//         y.push_back(vertex[1]);
//     }
//     x.push_back(vertices[0][0]);
//     y.push_back(vertices[0][1]);
//     matplot::plot(x, y, color);
// }
//
// void testRDT() {
//     Node2f* start = new Node2f(0, 0);
//     Node2f* goal = new Node2f(5, 7);
//
//     RDT<float, 2> rdt(start, goal);
//
//     // distribution
//     std::vector<std::pair<float, float>> intervals;
//     intervals.push_back(std::make_pair(0, 10));
//     intervals.push_back(std::make_pair(0, 10));
//     PointDistribution2f point_distribution(intervals);
//     rdt.setDistribution(point_distribution);
//
//     Point2f s1(0, 0);
//     Point2f s2(0, 1);
//     Point2f s3(1, 1);
//     Point2f s4(1, 0);
//     Polygon<float, 2> system({s1, s2, s3, s4});
//     rdt.setSystem(system);
//
//     // obstacles
//     Point2f p1(2, 2);
//     Point2f p2(2, 5);
//     Point2f p3(5, 5);
//     Point2f p4(5, 2);
//     Polygon<float, 2> poly({p1, p2, p3, p4});
//     Point2f p5(6, 6);
//     Point2f p6(6, 8);
//     Point2f p7(8, 6);
//     Polygon<float, 2> poly2({p5, p6, p7});
//
//     rdt.addObstacle(poly);
//     rdt.addObstacle(poly2);
//     bool is_solved = rdt.run();
//
//
//     if (!is_solved) {
//         printf("RDT failed to find a solution\n");
//         return;
//     }
//     std::vector<Node2f*> nodes;
//     rdt.getNodes(nodes);
//     std::vector<Polygon<float, 2>> obstacles;
//     rdt.getObstacles(obstacles);
//
//
//     // return;
//     Node2f* goal_node;
//     rdt.getGoal(goal_node);
//
//     // plotting
//     auto fig = matplot::figure(true);
//     matplot::hold(matplot::on);
//     std::vector<float> x;
//     std::vector<float> y;
//
//     // plot system at start
//     system.moveToPoint(*start);
//     std::string green = "g";
//     plotPolygon(system, green);
//
//     // plot system at end
//     system.moveToPoint(*goal);
//     plotPolygon(system, green);
//
//     // plot obstacles
//     std::string black = "k";
//     plotPolygon(poly, black);
//     plotPolygon(poly2, black);
//
//     x.clear();
//     y.clear();
//
//     for (auto node: nodes) {
//
//         x.clear();
//         y.clear();
//         x.push_back((*node)[0]);
//         y.push_back((*node)[1]);
//         if (node->parent != nullptr) {
//             x.push_back(node->parent->operator[](0));
//             y.push_back(node->parent->operator[](1));
//             // std::cout << "There is a parent" << std::endl;
//             matplot::plot(x, y, "b");
//         }
//     }
//
//     x.clear();
//     y.clear();
//     Node<float, 2> *n = goal_node;
//     while (n != nullptr) {
//         x.push_back(n->operator[](0));
//         y.push_back(n->operator[](1));
//         // std::cout << "x " << n->x << " y " << n->y << std::endl;
//         n = n->parent;
//     }
//     // x.push_back(start->x);
//     // y.push_back(start->y);
//     matplot::plot(x, y, "r");
//     matplot::xlim({-1, 11});
//     matplot::ylim({-1, 11});
//     matplot::grid(matplot::on);
//     matplot::show();
// }
//
// void testSeparatingAxis() {
//     Point2f p1(2, 2);
//     Point2f p2(2, 4);
//     Point2f p3(4, 4);
//     Point2f p4(4, 2);
//     Polygon<float, 2> poly({p1, p2, p3, p4});
//
//     Point2f p5(0, 1);
//     Point2f p6(1, 1);
//     Point2f p7(1, 0);
//     Point2f p8(0, 0);
//     Polygon<float, 2> poly2({p5, p6, p7, p8});
//     // poly2.printCentroid();
//     Point2f p0(2.5, 4.499);
//     poly2.moveToPoint(p0);
//     // poly2.printCentroid();
//
//     //
//     // std::vector<Point2f> v;
//     // poly2.getVertices(v);
//     // for (auto vertex: v) {
//     //     printf("vertex: %f %f\n", vertex[0], vertex[1]);
//     // }
//     // return;
//
//
//     // return;
//     bool is_colliding = areObjectsColliding(poly, poly2);
//     if (is_colliding)
//         printf("Polygons are colliding\n");
//     else
//         printf("Polygons are NOT colliding\n");
//
//     // return;
//     auto fig = matplot::figure(true);
//     matplot::hold(matplot::on);
//     std::vector<float> x;
//     std::vector<float> y;
//     std::vector<Point2f> vertices;
//     poly.getVertices(vertices);
//     for (auto vertex: vertices) {
//         x.push_back(vertex[0]);
//         y.push_back(vertex[1]);
//     }
//     x.push_back(vertices[0][0]);
//     y.push_back(vertices[0][1]);
//     matplot::plot(x, y, "k");
//
//     x.clear();
//     y.clear();
//     vertices.clear();
//     poly2.getVertices(vertices);
//     for (auto vertex: vertices) {
//         x.push_back(vertex[0]);
//         y.push_back(vertex[1]);
//     }
//     x.push_back(vertices[0][0]);
//     y.push_back(vertices[0][1]);
//     matplot::plot(x, y, "k");
//
//     matplot::xlim({-1, 11});
//     matplot::ylim({-1, 11});
//     matplot::grid(matplot::on);
//     matplot::show();
// }
//
// void plotFace(Polygon<float, 3>& face, std::string color) {
//     std::vector<float> x;
//     std::vector<float> y;
//     std::vector<float> z;
//     std::vector<Point<float, 3>> vertices;
//     face.getVertices(vertices);
//     for (auto vertex: vertices) {
//         x.push_back(vertex[0]);
//         y.push_back(vertex[1]);
//         z.push_back(vertex[2]);
//     }
//     x.push_back(vertices[0][0]);
//     y.push_back(vertices[0][1]);
//     z.push_back(vertices[0][2]);
//     auto pl = matplot::plot3(x, y, z);
//     pl->color(color);
// }
//
// void plotPolyhedra(Polyhedra<float> polyhedra, std::string color) {
//     std::vector<Polygon<float, 3>> faces;
//     polyhedra.getFaces(faces);
//
//     for (auto face: faces) {
//         plotFace(face, color);
//     }
// }
//
// void createCube(Polyhedra<float> &poly) {
//     Point<float, 3> p1{1, 0, 1};
//     Point<float, 3> p2{1, 1, 1};
//     Point<float, 3> p3{0, 1, 1};
//     Point<float, 3> p4{0, 0, 1};
//     Polygon<float, 3> face1 = Polygon<float, 3>({p1, p2, p3, p4});
//
//     p1 = Point<float, 3>{1, 0, 0};
//     p2 = Point<float, 3>(1, 1, 0);
//     p3 = Point<float, 3>(0, 1, 0);
//     p4 = Point<float, 3>(0, 0, 0);
//     Polygon<float, 3> face2 = Polygon<float, 3>({p1, p2, p3, p4});
//
//     p1 = Point<float, 3>(1, 0, 1);
//     p2 = Point<float, 3>(1, 1, 1);
//     p3 = Point<float, 3>(1, 1, 0);
//     p4 = Point<float, 3>(1, 0, 0);
//     Polygon<float, 3> face3 = Polygon<float, 3>({p1, p2, p3, p4});
//
//     p1 = Point<float, 3>(0, 0, 1);
//     p2 = Point<float, 3>(0, 1, 1);
//     p3 = Point<float, 3>(0, 1, 0);
//     p4 = Point<float, 3>(0, 0, 0);
//     Polygon<float, 3> face4 = Polygon<float, 3>({p1, p2, p3, p4});
//
//     p1 = Point<float, 3>(1, 1, 1);
//     p2 = Point<float, 3>(1, 1, 0);
//     p3 = Point<float, 3>(0, 1, 0);
//     p4 = Point<float, 3>(0, 1, 1);
//     Polygon<float, 3> face5 = Polygon<float, 3>({p1, p2, p3, p4});
//
//     p1 = Point<float, 3>(1, 0, 1);
//     p2 = Point<float, 3>(1, 0, 0);
//     p3 = Point<float, 3>(0, 0, 0);
//     p4 = Point<float, 3>(0, 0, 1);
//     Polygon<float, 3> face6 = Polygon<float, 3>({p1, p2, p3, p4});
//
//     Polyhedra<float> cube = Polyhedra<float>({face1, face2, face3, face4, face5, face6});
//     poly = cube;
// }
//
//
// void testPolyhedra() {
//     Point<float, 3> c{0, 0, 0};
//
//     Point<float, 3> p1{1, 0, 0.0};
//     Point<float, 3> p2{1, 2, 0.0};
//     Point<float, 3> p3{0, 1.2, 0.0};
//     Point<float, 3> p4{0, 0, 0.0};
//     std::vector<Point<float, 3>> points = {p1, p2, p3, p4};
//
//     // Point<float, 3> u = p2 - p1;
//     // Point<float, 3> v = p3 - p1;
//     // Point<float, 3> n = u.cross(v);
//     //
//     // float dot = n.dot(c - p1);
//     // if (dot > 0)
//     //     n = -n;
//     // n.normalize();
//     // printf("n: %f %f %f\n", n[0], n[1], n[2]);
//     //
//     // Point<float, 3> z_axis{0, 0, 1};
//     // Point<float, 3> axis = n.cross(z_axis);
//     // axis.normalize();
//     // float theta = std::acos(n.dot(z_axis));
//     //
//     // Eigen::Quaternionf q;
//     // q = Eigen::AngleAxisf(theta, axis);
//     // printf("q: %.3f %.3f %.3f %.3f\n", q.w(), q.x(), q.y(), q.z());
//     // // equivalent method
//     // dot = n.dot(z_axis);
//     // axis = n.cross(z_axis);
//     // float q_w = dot + std::sqrt(dot*dot + axis.dot(axis));
//     // q.w() = q_w;
//     // q.x() = axis[0];
//     // q.y() = axis[1];
//     // q.z() = axis[2];
//     // q.normalize();
//     // printf("q: %.3f %.3f %.3f %.3f\n", q.w(), q.x(), q.y(), q.z());
//     //
//     // q.setFromTwoVectors(n, z_axis);
//     // printf("q: %.3f %.3f %.3f %.3f\n", q.w(), q.x(), q.y(), q.z());
//     //
//     // // // rotate p1
//     // // Point<float, 3> p1_rotated = q * p1;
//     // // printf("p1_rotated: %f %f %f\n", p1_rotated[0], p1_rotated[1], p1_rotated[2]);
//     //
//     // std::vector<Point<float, 3>> vertices;
//     // vertices.push_back(p1);
//     // vertices.push_back(p2);
//     // vertices.push_back(p3);
//     // std::vector<Point<float, 3>> vertices_rotated;
//     // for (auto vertex: vertices) {
//     //     Point<float, 3> vertex_rotated = q * vertex;
//     //     vertices_rotated.push_back(vertex_rotated);
//     //     printf("vertex_rotated: %.3f %.3f %.3f\n", vertex_rotated[0], vertex_rotated[1], vertex_rotated[2]);
//     // }
//
//     Polyhedra<float> cube = Polyhedra<float>();
//     createCube(cube);
//     cube.moveToPoint(Point<float, 3>(0.0, 0.0, 0.0));
//
//     Polyhedra<float> cube2 = Polyhedra<float>();
//     createCube(cube2);
//     cube2.moveToPoint(Point<float, 3>(0.70, 0.20, 0.90));
//
//     bool in_collision = areObjectsColliding(cube, cube2);
//     if (in_collision)
//         printf("Polyhedra are colliding\n");
//     else
//         printf("Polyhedra are NOT colliding\n");
//
//
//     Point<float, 3> q(0.1, 0.2, 2.0);
//     in_collision = cube.isPointIncluded(q);
//     if (in_collision)
//         printf("Point is included\n");
//     else
//         printf("Point is NOT included\n");
//
//     return;
//
//
//     auto fig = matplot::figure(true);
//     matplot::hold(matplot::on);
//     std::vector<float> x;
//     std::vector<float> y;
//     std::vector<float> z;
//     std::vector<Point<float, 3>> vertices;
//     std::vector<Polygon<float, 3>> faces;
//     cube.getFaces(faces);
//     for (auto face: faces) {
//         plotFace(face, "k");
//     }
//
//     faces.clear();
//     cube2.getFaces(faces);
//     for (auto face: faces) {
//         plotFace(face, "b");
//     }
//
//     Point<float, 3> centroid_1, centroid_2;
//     cube.getCentroid(centroid_1);
//     // printf("centroid_1: %.3f %.3f %.3f\n", centroid_1[0], centroid_1[1], centroid_1[2]);
//     x.push_back(centroid_1[0]);
//     y.push_back(centroid_1[1]);
//     z.push_back(centroid_1[2]);
//     matplot::plot3(x, y, z, "r*");
//     x.clear();
//     y.clear();
//     z.clear();
//     cube2.getCentroid(centroid_2);
//     // printf("centroid_2: %.3f %.3f %.3f\n", centroid_2[0], centroid_2[1], centroid_2[2]);
//     x.push_back(centroid_2[0]);
//     y.push_back(centroid_2[1]);
//     z.push_back(centroid_2[2]);
//     matplot::plot3(x, y, z, "b*");
//
//     matplot::xlim({-4, 4});
//     matplot::ylim({-4, 4});
//     matplot::zlim({-4, 4});
//     matplot::xlabel("x");
//     matplot::ylabel("y");
//     matplot::zlabel("z");
//     matplot::show();
// }
//
// void test3DRDT() {
//     Node3f* start = new Node3f(0, 0, 0);
//     Node3f* goal = new Node3f(5, 7, 5);
//
//     RDT<float, 3> rdt(start, goal);
//
//     // distribution
//     std::vector<std::pair<float, float>> intervals;
//     intervals.push_back(std::make_pair(0, 10));
//     intervals.push_back(std::make_pair(0, 10));
//     intervals.push_back(std::make_pair(0, 10));
//     PointDistribution3f point_distribution(intervals);
//     rdt.setDistribution(point_distribution);
//
//     Polyhedra<float> system;
//     createCube(system);
//     system.moveToPoint(Point<float, 3>(0, 0, 0));
//     rdt.setSystem(system);
//
//     // obstacles
//     Polyhedra<float> obstacle1;
//     createCube(obstacle1);
//     obstacle1.moveToPoint(Point<float, 3>(2, 2, 2));
//     rdt.addObstacle(obstacle1);
//
//     // rdt.addObstacle(poly2);
//     bool is_solved = rdt.run();
//
//     // plot system at start
//     auto fig = matplot::figure(true);
//     matplot::hold(matplot::on);
//
//     system.moveToPoint(*start);
//     plotPolyhedra(system, "g");
//
//     // plot system at end
//     system.moveToPoint(*goal);
//     plotPolyhedra(system, "g");
//
//     // plot obstacles
//     plotPolyhedra(obstacle1, "k");
//
//     // plot nodes
//     std::vector<Node3f*> nodes;
//     rdt.getNodes(nodes);
//     std::vector<float> x;
//     std::vector<float> y;
//     std::vector<float> z;
//     for (auto node: nodes) {
//         x.clear();
//         y.clear();
//         z.clear();
//         x.push_back((*node)[0]);
//         y.push_back((*node)[1]);
//         z.push_back((*node)[2]);
//         if (node->parent != nullptr) {
//             x.push_back(node->parent->operator[](0));
//             y.push_back(node->parent->operator[](1));
//             z.push_back(node->parent->operator[](2));
//             // std::cout << "There is a parent" << std::endl;
//             matplot::plot3(x, y, z, "b");
//         }
//     }
//
//     // plot path
//     x.clear();
//     y.clear();
//     z.clear();
//     Node<float, 3> *n;
//     rdt.getGoal(n);
//
//     while (n != nullptr) {
//         x.push_back(n->operator[](0));
//         y.push_back(n->operator[](1));
//         z.push_back(n->operator[](2));
//         // std::cout << "x " << n->x << " y " << n->y << std::endl;
//         n = n->parent;
//     }
//     matplot::plot3(x, y, z, "r");
//
//     matplot::xlim({-1, 11});
//     matplot::ylim({-1, 11});
//     matplot::zlim({-1, 11});
//
//     matplot::grid(matplot::on);
//     matplot::show();
//
// }
//
// void testDijkstra() {
//     Node2f* start = new Node2f(0, 0);
//     Node2f* goal = new Node2f(5, 7);
//
//     // Dijkstra<float, 2> discrete_search = Dijkstra<float, 2>(start, goal);
//     AStar<float, 2> discrete_search = AStar<float, 2>(start, goal);
//
//     Point2f s1(0, 0);
//     Point2f s2(0, 0.5);
//     Point2f s3(0.5, 0.5);
//     Point2f s4(0.5, 0);
//     Polygon<float, 2> system = Polygon<float, 2>({s1, s2, s3, s4});
//     discrete_search.setSystem(system);
//
//     Point2f o1(0, 4);
//     Point2f o2(0, 5.5);
//     Point2f o3(5.5, 5.5);
//     Point2f o4(5.5, 0);
//     Polygon<float, 2> obstacle = Polygon<float, 2>({o1, o2, o3, o4});
//     discrete_search.addObstacle(obstacle);
//
//     // boundaries
//     std::vector<std::pair<float, float>> limits;
//     limits.emplace_back(-1, 10);
//     limits.emplace_back(-1, 10);
//     discrete_search.setSpaceLimits(limits);
//
//     discrete_search.run();
//
//     bool is_solution_found = discrete_search.isSolutionFound();
//     if (!is_solution_found) {
//         printf("Dijkstra failed to find a solution\n");
//         return;
//     }
//
//     std::vector<Node2f*> nodes;
//     discrete_search.getSolution(nodes);
//
//     auto fig = matplot::figure(true);
//
//     auto ax = fig->add_subplot(1, 1, 1);
//     // matplot::hold(matplot::on);
//
//
//
//     // // plot system at start
//
//     // system.moveToPoint(*start);
//     // std::string green = "g";
//     // plotPolygon(system, green);
//     // // fig->draw();
//     //
//     for (auto node: nodes) {
//         ax->clear();
//         ax->hold(matplot::on);
//         ax->grid(matplot::on);
//         ax->xlim({-1, 11});
//         ax->ylim({-1, 11});
//
//         system.moveToPoint(*node);
//         plotPolygon(system, "b");
//         plotPolygon(obstacle, "k");
//
//         ax->draw();
//
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));
//
//     }
//
//     // // plot system at start
//     // system.moveToPoint(*start);
//     // std::string green = "g";
//     // plotPolygon(system, green);
//     //
//     // // plot system at goal
//     // system.moveToPoint(*goal);
//     // plotPolygon(system, green);
//     //
//     // // plot obstacles
//     // std::string black = "k";
//     // plotPolygon(obstacle, black);
//     //
//     // std::vector<float> x;
//     // std::vector<float> y;
//     // for (auto node: nodes) {
//     //     x.clear();
//     //     y.clear();
//     //     x.push_back((*node)[0]);
//     //     y.push_back((*node)[1]);
//     //     if (node->parent != nullptr) {
//     //         x.push_back(node->parent->operator[](0));
//     //         y.push_back(node->parent->operator[](1));
//     //         // std::cout << "There is a parent" << std::endl;
//     //         matplot::plot(x, y, "b");
//     //     }
//     // }
//     //
//     // matplot::xlim({-1, 11});
//     // matplot::ylim({-1, 11});
//     // matplot::grid(matplot::on);
//     // matplot::show();
//
//
//
//
//
//
// }
//
// int main() {
//
//     // Node2d n1(1, 1);
//     // Node2d n2(1, 6);
//     // Node2d n3(6, 6);
//     // Point2d n4(6, 1);
//     //
//     // Polygon<double> poly({n1, n2, n3, n4});
//     // Node2d n5(3, 5);
//     //
//     // bool is_included = poly.isPointIncluded(n5);
//     // if (is_included)
//     //     printf("Point is included\n");
//     // else
//     //     printf("Point is not included\n");
//     // testRDT();
//     // testSeparatingAxis();
//     // testPolyhedra();
//     // test3DRDT();
//     testDijkstra();
//
//
//     return 0;
// }



#include <cstdio>
#include "XiPlanningAlgorithms/Dijkstra.hpp"


int main() {
    printf("Example Dijkstra\n");

    Node2d n1(1, 1);
    printf("n1: %f %f\n", n1[0], n1[1]);

    return 0;
}