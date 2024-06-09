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


#ifndef XI_PLANNING_ALGORITHMS_CORE_HPP
#define XI_PLANNING_ALGORITHMS_CORE_HPP

#include <random>
#include <cassert>
#include <vector>
#include <eigen3/Eigen/Dense>


// Point
template <typename T, size_t dim>
struct Point : public Eigen::Vector<T, dim> {
    using Eigen::Vector<T, dim>::Vector;

    Point() {this->setZero();}

    using value_type = T;
    static constexpr size_t dimension = dim;

    T& x() {return this->coeffRef(0);}
    T& y() {return this->coeffRef(1);}
    T& z() {return this->coeffRef(2);}
};

// Edge
template <typename T, size_t dim>
struct Edge {
    Point<T, dim> a;
    Point<T, dim> b;

    Edge() = default;
    Edge(Point<T, dim> a, Point<T, dim> b) : a(a), b(b) {}
};

// Node
template <typename T, size_t dim>
struct Node : public Point<T, dim> {
    using Point<T, dim>::Point;

    T cost;
    T cost_to_come;
    T cost_to_go;
    Node<T, dim> *parent {nullptr};
    T eps {1e-6};
    bool is_visited {false};

    bool operator == (const Node<T, dim>* rhs) const {
        return (*this - *rhs).norm() < eps;
    }

    bool isVisited() const {
        return is_visited;
    }

    void setVisited(bool flag) {
        is_visited = flag;
    }

};

// PointDistribution
template <typename T, size_t dim>
struct PointDistribution {

    std::mt19937 rnd_gen{std::random_device{}()};

    PointDistribution() = default;

    explicit PointDistribution(std::vector<std::pair<T, T>> intervals) {
        assert(("Dimensions are not valid", intervals.size() == dim));
        dists.reserve(dim);

        for (auto &interval: intervals) {
            dists.emplace_back(std::uniform_real_distribution<>(interval.first, interval.second));
        }
    }

    std::vector<std::uniform_real_distribution<>> dists;

    Point<T, dim> sample() {
        Point<T, dim> point;
        for (int i=0; i<dim; i++) {
            point[i] = dists[i](rnd_gen);
        }
        return point;
    }

    void sample(Point<T, dim>& point) {
        for (int i=0; i<dim; i++)
            point[i] = dists[i](rnd_gen);
    }

    void sample(Node<T, dim>& node) {
        for (int i=0; i<dim; i++)
            node[i] = dists[i](rnd_gen);
    }

};

// 2D
typedef Point<float, 2> Point2f;
typedef Point<double, 2> Point2d;
typedef Node<float, 2> Node2f;
typedef Node<double, 2> Node2d;
typedef Edge<double, 2> Edge2f;
typedef Edge<double, 2> Edge2d;
typedef PointDistribution<float, 2> PointDistribution2f;
typedef PointDistribution<double, 2> PointDistribution2d;

// 3D
typedef Point<float, 3> Point3f;
typedef Point<double, 3> Point3d;
typedef Node<float, 3> Node3f;
typedef Node<double, 3> Node3d;
typedef Edge<double, 3> Edge3f;
typedef Edge<double, 3> Edge3d;
typedef PointDistribution<float, 3> PointDistribution3f;
typedef PointDistribution<double, 3> PointDistribution3d;

#endif //XI_PLANNING_ALGORITHMS_CORE_HPP
