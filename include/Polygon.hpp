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


#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <vector>

#include "Geometry.hpp"
#include "utils.hpp"

template <typename T>
class Polygon {

    static constexpr size_t dim = 2;

public:
    Polygon() = default;
    explicit Polygon(std::vector<Point<T, dim>> vertices);

    bool isPointIncluded(Point<T, dim>& point);
    void getEdges(std::vector<Edge<T>>& edges) {edges = edges_;}
    void getVertices(std::vector<Point<T, dim>>& vertices) {vertices = vertices_;}

private:

    void orderVertices();
    void createEdges();

    std::vector<Point<T, dim>> vertices_;
    std::vector<Edge<T>> edges_;


};


// Polygon(std::vector<Point<T, dim>> vertices)
template <typename T>
Polygon<T>::Polygon(std::vector<Point<T, dim>> vertices) {
    vertices_ = vertices;
    orderVertices();
    createEdges();
}

// void orderVertices()
template <typename T>
void Polygon<T>::orderVertices() {
    size_t n = vertices_.size();
    Point<T, dim> centroid;
    for (auto &vertex: vertices_) {
        centroid += vertex;
    }
    centroid /= n;

    std::vector<std::pair<size_t, T>> angles;
    angles.reserve(n);
    for (size_t i=0; i<n; i++) {
        angles.emplace_back(
            i, atan2(vertices_[i].y() - centroid.y(), vertices_[i].x() - centroid.x())
            );
    }

    std::sort(angles.begin(), angles.end(), [](auto &left, auto &right) {
        return left.second < right.second;
    });

    std::vector<Point<T, dim>> ordered_vertices;
    for (size_t i=0; i<n; i++) {
        ordered_vertices.push_back(vertices_[angles[i].first]);
    }
    vertices_ = ordered_vertices;
}

// void createEdges()
template <typename T>
void Polygon<T>::createEdges() {
    edges_.clear();
    size_t n = vertices_.size();
    for (size_t i=0; i<n-1; i++) {
        Edge<T> edge{};
        edge.a = vertices_[i];
        edge.b = vertices_[i+1];
        edges_.push_back(edge);
    }
    Edge<T> edge{};
    edge.a = vertices_.back();
    edge.b = vertices_.front();
    edges_.push_back(edge);
}

// bool isPointIncluded(Point<T, dim>& point)
template <typename T>
bool Polygon<T>::isPointIncluded(Point<T, dim>& point) {
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


#endif //POLYGON_HPP
