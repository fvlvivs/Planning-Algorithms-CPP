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


#ifndef XI_PLANNING_ALGORITHMS_POLYGON_HPP
#define XI_PLANNING_ALGORITHMS_POLYGON_HPP

#include <vector>

#include "Core.hpp"
#include "utils.hpp"


template <typename T, size_t dim>
class Polygon {

public:
    Polygon() = default;
    explicit Polygon(std::vector<Point<T, dim>> vertices);

    bool isPointIncluded(Point<T, dim>& point);
    void getEdges(std::vector<Edge<T, dim>>& edges) {edges = edges_;}
    void getVertices(std::vector<Point<T, dim>>& vertices) {vertices = vertices_;}
    void getCentroid(Point<T, dim>& centroid) {centroid = centroid_;}
    void getFirstVertex(Point<T, dim>& vertex) {vertex = vertices_[0];}
    void moveToPoint(Point<T, dim> point);
    void shiftByDelta(Point<T, dim> delta);
    void getNormal(Point<T, dim>& normal);
    void getNormalWithRespectToPoint(Point<T, dim>& point, Point<T, dim>& normal);

    void orderVertices();  // 2D
    void orderVertices(Point<T, dim> reference);  // 3D

private:
    void createEdges();

    std::vector<Point<T, dim>> vertices_;
    std::vector<Edge<T, dim>> edges_;
    Point<T, dim> centroid_;
};


// Polygon(std::vector<Point<T, dim>> vertices)
template <typename T, size_t dim>
Polygon<T, dim>::Polygon(std::vector<Point<T, dim>> vertices) {
    vertices_ = vertices;
    orderVertices();
    createEdges();
}

// void createEdges()
template <typename T, size_t dim>
void Polygon<T, dim>::createEdges() {
    edges_.clear();
    size_t n = vertices_.size();
    for (size_t i=0; i<n-1; i++) {
        Edge<T, dim> edge{};
        edge.a = vertices_[i];
        edge.b = vertices_[i+1];
        edges_.push_back(edge);
    }
    Edge<T, dim> edge{};
    edge.a = vertices_.back();
    edge.b = vertices_.front();
    edges_.push_back(edge);
}

// bool isPointIncluded(Point<T, dim>& point)
template <typename T, size_t dim>
bool Polygon<T, dim>::isPointIncluded(Point<T, dim>& point) {
    size_t low = 0;
    size_t N = vertices_.size();
    size_t high = N;

    while (low + 1 < high) {
        size_t mid = (low + high) / 2;
        if (isTriangleCCW(vertices_[0], vertices_[mid], point))
            low = mid;
        else
            high = mid;
    }

    if (low == 0 || high == N)
        return false;

    return isTriangleCCW(vertices_[low], vertices_[high], point);
}

// void moveToPoint(Point<T, dim>& point)
template <typename T, size_t dim>
void Polygon<T, dim>::moveToPoint(Point<T, dim> point) {
    Point<T, dim> delta = point - centroid_;
    centroid_ = point;
    for (auto &vertex: vertices_)
        vertex += delta;

    createEdges();
}

// void shiftByDelta(Point<T, dim>& delta)
template <typename T, size_t dim>
void Polygon<T, dim>::shiftByDelta(Point<T, dim> delta) {
    centroid_ += delta;
    for (auto &vertex: vertices_)
        vertex += delta;

    createEdges();
}

// void getNormal(Point<T, dim>& normal)
template <typename T, size_t dim>
void Polygon<T, dim>::getNormal(Point<T, dim>& normal) {
    if (dim == 2)
        throw std::runtime_error("Polygon::getNormal() is not defined for 2D polygons");

    getPlaneNormal(vertices_[0], vertices_[1], vertices_[2], normal);
}

// void getNormalWithRespectToPoint(Point<T, dim>& point, Point<T, dim>& normal)
template <typename T, size_t dim>
void Polygon<T, dim>::getNormalWithRespectToPoint(Point<T, dim>& point, Point<T, dim>& normal) {
    getNormal(normal);
    T dot = normal.dot(point - vertices_[0]);
    if (dot > 0)
        normal = -normal;
}

// void orderVertices() [2D]
template <typename T, size_t dim>
void Polygon<T, dim>::orderVertices() {
    size_t n = vertices_.size();
    centroid_ = Point<T, dim>::Zero();
    for (auto &vertex: vertices_) {
        centroid_ += vertex;
    }
    centroid_ /= n;

    std::vector<std::pair<size_t, T>> angles;
    angles.reserve(n);
    for (size_t i = 0; i < n; i++) {
        angles.emplace_back(
                i, atan2(vertices_[i].y() - centroid_.y(), vertices_[i].x() - centroid_.x())
        );
    }

    std::sort(angles.begin(), angles.end(), [](auto &left, auto &right) {
        return left.second > right.second;
    });

    std::vector<Point<T, dim>> ordered_vertices;
    for (size_t i = 0; i < n; i++) {
        ordered_vertices.push_back(vertices_[angles[i].first]);
    }
    vertices_ = ordered_vertices;
}


// void orderVertices() [3D]
template <typename T, size_t dim>
void Polygon<T, dim>::orderVertices(Point<T, dim> reference) {

    Point<T, dim> normal;
    getPlaneNormal(vertices_[0], vertices_[1], vertices_[2], normal);
    T dot = normal.dot(reference - vertices_[0]);
    // make them point in the same direction
    if (dot > 0)
        normal = -normal;

    // find the quaternion that rotates normal to z_axis
    Point<T, dim> z_axis{0, 0, 1};
    Eigen::Quaternion<T> q;
    q.setFromTwoVectors(normal, z_axis);

    // rotate all vertices
    for (auto& vertex: vertices_)
        vertex = q * vertex;

    // now that are all in the same plane, order them counter-clockwise
    // find the centroid (tmp rotated)
    Point<T, dim> centroid = Point<T, dim>::Zero();
    for (auto vertex: vertices_)
        centroid += vertex;
    centroid /= vertices_.size();

    std::vector<std::pair<size_t, T>> angles;
    angles.reserve(vertices_.size());
    for (size_t i=0; i<vertices_.size(); i++) {
        angles.emplace_back(
                i, atan2(vertices_[i].y() - centroid.y(), vertices_[i].x() - centroid.x())
                );
    }

    std::sort(angles.begin(), angles.end(), [](auto& a, auto& b) {
        return a.second > b.second;
    });

    std::vector<Point<T, dim>> vertices_ordered;
    for (size_t i=0; i<vertices_.size(); i++) {
        vertices_ordered.push_back(q.conjugate() * vertices_[angles[i].first]);
    }
    vertices_ = vertices_ordered;

    // compute centroid
    centroid_ = Point<T, dim>::Zero();
    for (auto vertex: vertices_)
        centroid_ += vertex;
    centroid_ /= vertices_.size();
}

#endif //XI_PLANNING_ALGORITHMS_POLYGON_HPP
