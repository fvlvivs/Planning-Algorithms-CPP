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


#ifndef XI_PLANNING_ALGORITHMS_PLOTUTILS_HPP
#define XI_PLANNING_ALGORITHMS_PLOTUTILS_HPP

#include <XiPlanningAlgorithms/Core.hpp>
#include <XiPlanningAlgorithms/Polygon.hpp>
#include <XiPlanningAlgorithms/Polyhedra.hpp>

#include <matplot/matplot.h>

template <typename T, size_t dim>
void plotPolygon(Polygon<T, 2>& poly, std::string color) {
    std::vector<T> x;
    std::vector<T> y;
    std::vector<Point<T, 2>> vertices;
    poly.getVertices(vertices);
    for (auto vertex: vertices) {
        x.push_back(vertex[0]);
        y.push_back(vertex[1]);
    }
    x.push_back(vertices[0][0]);
    y.push_back(vertices[0][1]);
    auto pl = matplot::plot(x, y);
    pl->color(color);
}

template <typename T, size_t dim>
void plotFace(Polygon<T, 3>& face, std::string color) {
    std::vector<T> x;
    std::vector<T> y;
    std::vector<T> z;
    std::vector<Point<T, 3>> vertices;
    face.getVertices(vertices);
    for (auto vertex: vertices) {
        x.push_back(vertex[0]);
        y.push_back(vertex[1]);
        z.push_back(vertex[2]);
    }
    x.push_back(vertices[0][0]);
    y.push_back(vertices[0][1]);
    z.push_back(vertices[0][2]);
    auto pl = matplot::plot3(x, y, z);
    pl->color(color);
}

template <typename T>
void plotPolyhedra(Polyhedra<T> polyhedra, std::string color) {
    std::vector<Polygon<T, 3>> faces;
    polyhedra.getFaces(faces);

    for (auto face: faces) {
        plotFace<T, 3>(face, color);
    }
}

template <typename T, size_t dim>
void plotPath(std::vector<Node<T, dim>*> path, std::string color) {
    std::vector<T> x;
    std::vector<T> y;
    std::vector<T> z;
    for (auto node: path) {
        x.push_back(node->operator[](0));
        y.push_back(node->operator[](1));
        if (dim == 3)
            z.push_back(node->operator[](2));
    }

    if (dim == 2) {
        auto pl = matplot::plot(x, y);
        pl->color(color);
    } else {
        auto pl = matplot::plot3(x, y, z);
        pl->color(color);
    }
}

template <typename T>
void createParallelepiped(Polyhedra<T> &poly, T width, T height, T depth) {
    Point<T, 3> p1(width, 0, depth);
    Point<T, 3> p2(width, height, depth);
    Point<T, 3> p3(0, height, depth);
    Point<T, 3> p4(0, 0, depth);
    Polygon<T, 3> face1 = Polygon<T, 3>({p1, p2, p3, p4});

    p1 = Point<T, 3>{width, 0, 0};
    p2 = Point<T, 3>(width, height, 0);
    p3 = Point<T, 3>(0, height, 0);
    p4 = Point<T, 3>(0, 0, 0);
    Polygon<T, 3> face2 = Polygon<T, 3>({p1, p2, p3, p4});

    p1 = Point<T, 3>(width, 0, depth);
    p2 = Point<T, 3>(width, height, depth);
    p3 = Point<T, 3>(width, height, 0);
    p4 = Point<T, 3>(width, 0, 0);
    Polygon<T, 3> face3 = Polygon<T, 3>({p1, p2, p3, p4});

    p1 = Point<T, 3>(0, 0, depth);
    p2 = Point<T, 3>(0, height, depth);
    p3 = Point<T, 3>(0, height, 0);
    p4 = Point<T, 3>(0, 0, 0);
    Polygon<T, 3> face4 = Polygon<T, 3>({p1, p2, p3, p4});

    p1 = Point<T, 3>(width, height, depth);
    p2 = Point<T, 3>(width, height, 0);
    p3 = Point<T, 3>(0, height, 0);
    p4 = Point<T, 3>(0, height, depth);
    Polygon<T, 3> face5 = Polygon<T, 3>({p1, p2, p3, p4});

    p1 = Point<T, 3>(width, 0, depth);
    p2 = Point<T, 3>(width, 0, 0);
    p3 = Point<T, 3>(0, 0, 0);
    p4 = Point<T, 3>(0, 0, depth);
    Polygon<T, 3> face6 = Polygon<T, 3>({p1, p2, p3, p4});

    Polyhedra<T> cube = Polyhedra<T>({face1, face2, face3, face4, face5, face6});
    poly = cube;
}

template <typename T, size_t dim>
void plotRDTNodes(std::vector<Node<T, dim>*> nodes, std::string color) {
    std::vector<T> x;
    std::vector<T> y;
    std::vector<T> z;

    for (auto node: nodes) {
        x.clear();
        y.clear();
        z.clear();

        x.push_back(node->operator[](0));
        y.push_back(node->operator[](1));
        if (dim == 3)
            z.push_back(node->operator[](2));

        if (node->parent != nullptr) {
            x.push_back(node->parent->operator[](0));
            y.push_back(node->parent->operator[](1));
            if (dim == 3)
                z.push_back(node->parent->operator[](2));
        }

        if (dim == 2) {
            auto pl = matplot::plot(x, y);
            pl->color(color);

        } else {
            auto pl = matplot::plot3(x, y, z);
            pl->color(color);
        }
    }

}



#endif //XI_PLANNING_ALGORITHMS_PLOTUTILS_HPP
