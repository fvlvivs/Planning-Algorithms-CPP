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


#ifndef XI_PLANNING_ALGORITHMS_POLYHEDRA_HPP
#define XI_PLANNING_ALGORITHMS_POLYHEDRA_HPP

#include "Core.hpp"
#include "Polygon.hpp"
#include "utils.hpp"


template <typename T>
class Polyhedra {
    static constexpr size_t dim = 3;


public:
    Polyhedra() = default;
    ~Polyhedra() = default;
    explicit Polyhedra(std::vector<Polygon<T, dim>> faces);
    void getFaces(std::vector<Polygon<T, dim>>& faces) {faces = faces_;}
    bool isPointIncluded(Point<T, dim>& point);

    void getCentroid(Point<T, dim>& centroid) {centroid = centroid_;}
    void moveToPoint(Point<T, dim> point);
    void getVertices(std::vector<Point<T, dim>>& vertices) {
        vertices.clear();
        for (auto& face: faces_) {
            std::vector<Point<T, dim>> face_vertices;
            face.getVertices(face_vertices);
            vertices.insert(vertices.end(), face_vertices.begin(), face_vertices.end());
        }
    }

    Point<T, dim> centroid_;
private:
    std::vector<Polygon<T, dim>> faces_;
    std::vector<Point<T, dim>> vertices_;
};

template <typename T>
Polyhedra<T>::Polyhedra(std::vector<Polygon<T, dim>> faces) {
    faces_ = faces;
    // find the centroid
    Point<T, dim> face_centroid;
    for (auto& face: faces_) {
        face.getCentroid(face_centroid);
        centroid_ += face_centroid;
    }
    centroid_ /= faces_.size();

    // order vertices of each face
    for (auto& face: faces_)
        face.orderVertices(centroid_);
}

template <typename T>
void Polyhedra<T>::moveToPoint(Point<T, dim> point) {
    Point<T, dim> delta = point - centroid_;
    centroid_ = point;
    for (auto& face: faces_)
        face.shiftByDelta(delta);
}

template <typename T>
bool Polyhedra<T>::isPointIncluded(Point<T, dim> &point) {
    bool condition = true;
    Point<T, dim> normal_to_centroid;
    Point<T, dim> normal_to_point;
    for (auto& face: faces_) {
        face.getNormalWithRespectToPoint(centroid_, normal_to_centroid);
        face.getNormalWithRespectToPoint(point, normal_to_point);
        condition = condition && (normal_to_centroid.dot(normal_to_point) > 0);
    }
    return condition;
}


#endif //XI_PLANNING_ALGORITHMS_POLYHEDRA_HPP
