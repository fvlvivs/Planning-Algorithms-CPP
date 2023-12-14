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

// Using Separating Axis Theorem to check if two polygons are colliding

// arePolygonsColliding(Polygon<T> &polygon_a, Polygon<T> &polygon_b)
template<typename T>
bool arePolygonsColliding(Polygon<T> &polygon_a, Polygon<T> &polygon_b) {
    std::vector<Edge<T, 2>> edges;
    // check with respect to first polygon
    polygon_a.getEdges(edges);
    for (auto& edge: edges) {
        Point<T, 2> diff = edge.b - edge.a;
        Point<T, 2> normal(diff.y(), -diff.x());
        if (!doesPolygonCrossAxis(edge.a, normal, polygon_b))
            return false;
    }

    // check with respect to second polygon
    edges.clear();
    polygon_b.getEdges(edges);
    for (auto& edge: edges) {
        Point<T, 2> diff = edge.b - edge.a;
        Point<T, 2> normal(diff.y(), -diff.x());
        if (!doesPolygonCrossAxis(edge.a, normal, polygon_a))
            return false;
    }
    return true;
}

// doesPolygonCrossAxis(Point<T, 2> &point, Point<T, 2> &axis, Polygon<T> &polygon)
template<typename T>
bool doesPolygonCrossAxis(Point<T, 2> &point, Point<T, 2> &axis, Polygon<T> &polygon) {
    std::vector<Point<T, 2>> vertices;
    polygon.getVertices(vertices);
    bool positive = false;
    bool negative = false;
    size_t positive_count = 0;
    for (auto& vertex: vertices) {
        T dot_product = axis.dot(vertex - point);
        if (dot_product > 0) {
            positive = true;
            positive_count++;
        } else if (dot_product < 0) {
            negative = true;
        }

        if (positive && negative)
            return true;
    }
    return positive_count > 0;
}


#endif //XI_PLANNING_ALGORITHMS_SEPARATINGAXIS_HPP
