
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <random>
#include <cassert>
#include <vector>
#include <Eigen/Dense>


// Point
template <typename T, size_t dim>
struct Point : public Eigen::Vector<T, dim> {
    using Eigen::Vector<T, dim>::Vector;

    Point() {this->setZero();}

    using value_type = T;
    static constexpr size_t dimension = dim;
};

// Edge
template <typename T>
struct Edge {
    T a;
    T b;

    Edge() = default;
    Edge(T a, T b) : a(a), b(b) {}
};

// Node
template <typename T, size_t dim>
struct Node : public Point<T, dim> {
    using Point<T, dim>::Point;

    T cost;
    T cost_to_come;
    T cost_to_go;
    Node<T, dim> *parent {nullptr};
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
typedef Edge<Node<double, 2>> Edge2f;
typedef Edge<Node<double, 2>> Edge2d;
typedef PointDistribution<float, 2> PointDistribution2f;
typedef PointDistribution<double, 2> PointDistribution2d;





#endif //GEOMETRY_HPP
