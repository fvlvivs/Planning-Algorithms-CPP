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

//
// Created by fulvius on 09/12/23.
//

#ifndef XI_PLANNING_ALGORITHMS_SEPARATINGAXIS_HPP
#define XI_PLANNING_ALGORITHMS_SEPARATINGAXIS_HPP

#include "Geometry.hpp"
#include "Polygon.hpp"

// Using Separating Axis Theorem to check if two polygons/polyhedra are colliding

// arePolygonsColliding(Polygon<T> &polygon_a, Polygon<T> &polygon_b)
template<typename T>
bool arePolygonsColliding(Polygon<T, 2> &polygon_a, Polygon<T, 2> &polygon_b) {
    std::vector<Edge<T, 2>> edges;
    // check with respect to first polygon
    polygon_a.getEdges(edges);
    for (auto& edge: edges) {
        Point<T, 2> diff = -(edge.b - edge.a);
        Point<T, 2> normal(diff.y(), -diff.x());
        if (computeProjectionSides(edge.a, normal, polygon_b) > 0)
            return false;
    }

    // check with respect to second polygon
    edges.clear();
    polygon_b.getEdges(edges);
    for (auto& edge: edges) {
        Point<T, 2> diff = -(edge.b - edge.a);
        Point<T, 2> normal(diff.y(), -diff.x());
        if (computeProjectionSides(edge.a, normal, polygon_a) > 0)
            return false;
    }
    return true;
}

template <typename T, size_t dim>
int computeProjectionSides(Point<T, dim> &point, Point<T, dim> &axis, Polygon<T, dim> &polygon) {
    std::vector<Point<T, dim>> vertices;
    polygon.getVertices(vertices);
    int positive_count = 0;
    int negative_count = 0;
    for (auto& vertex: vertices) {
        T dot_product = axis.dot(vertex - point);
        if (dot_product > 0) {
            positive_count++;
        } else if (dot_product < 0) {
            negative_count++;
        }

        if (positive_count > 0 && negative_count > 0)
            return 0;
    }

    return (positive_count > 0 ? 1 : -1);
}

template <typename T>
bool arePolyhedraColliding(Polyhedra<T> &polyhedra_a, Polyhedra<T> &polyhedra_b) {
    std::vector<Polygon<T, 3>> faces_a;
    std::vector<Polygon<T, 3>> faces_b;
    polyhedra_a.getFaces(faces_a);
    polyhedra_b.getFaces(faces_b);

    Point<T, 3> reference_point;
    // check with respect to first polyhedra
    polyhedra_a.getCentroid(reference_point);
    for (auto& face_a: faces_a) {
        Point<T, 3> normal;
        face_a.getNormalWithRespectToPoint(reference_point, normal);
        Point<T, 3> first_vertex;
        face_a.getFirstVertex(first_vertex);

        bool condition = false;
        for (auto& face_b: faces_b)
            condition = condition && (computeProjectionSides(first_vertex, normal, face_b) > 0);

        if (condition)
            return false;
    }

    polyhedra_b.getCentroid(reference_point);
    for (auto& face_b: faces_b) {
        Point<T, 3> normal;
        face_b.getNormalWithRespectToPoint(reference_point, normal);
        Point<T, 3> first_vertex;
        face_b.getFirstVertex(first_vertex);

        bool condition = false;
        for (auto& face_a: faces_a)
            condition = condition && (computeProjectionSides(first_vertex, normal, face_a) > 0);

        if (condition)
            return false;
    }

    for (auto& face_a: faces_a) {
        std::vector<Edge<T, 3>> edges_a;
        face_a.getEdges(edges_a);
        for (auto &edge_a: edges_a) {
            Point<T, 3> diff_a = edge_a.b - edge_a.a;
            Point<T, 3> p_a = edge_a.a;

            for (auto& face_b: faces_b) {
                std::vector<Edge<T, 3>> edges_b;
                face_b.getEdges(edges_b);
                for (auto &edge_b: edges_b) {
                    Point<T, 3> diff_b = edge_b.b - edge_b.a;
                    Point<T, 3> cross = diff_a.cross(diff_b);

                    if (cross.norm() > 1e-6) {
                        int side_a = 0;
                        for (auto& el_a: faces_a)
                            side_a += computeProjectionSides(p_a, cross, el_a);
                        if (side_a == 0)
                            continue;

                        int side_b = 0;
                        for (auto& el_b: faces_b)
                            side_b += computeProjectionSides(p_a, cross, el_b);
                        if (side_b == 0)
                            continue;

                        if (side_a * side_b < 0)
                            return false;
                    }
                }
            }
        }
    }
    return true;
}


#endif //XI_PLANNING_ALGORITHMS_SEPARATINGAXIS_HPP
