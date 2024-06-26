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


#ifndef PLANNING_ALGORITHMS_CPP_RDT_HPP
#define PLANNING_ALGORITHMS_CPP_RDT_HPP

#include <vector>
#include <type_traits>

#include "utils.hpp"
#include "Core.hpp"
#include "SeparatingAxis.hpp"


template <typename T, size_t dim>
class RDT {
    typedef Point<T, dim> PointT;
    typedef Node<T, dim> NodeT;
    typedef PointDistribution<T, dim> PointDistributionT;
    typedef typename std::conditional<
            dim == 2,
            Polygon<T, dim>,
            Polyhedra<T>
    >::type ObstacleT;
    typedef typename std::conditional<
            dim == 2,
            Polygon<T, dim>,
            Polyhedra<T>
    >::type SystemT;
    using Metric = T(*)(NodeT*, NodeT*);

public:
    RDT() = default;
    ~RDT() = default;
    explicit RDT(NodeT* start, NodeT* goal);
    void run();
    bool isSolutionFound() const {return is_solution_found_;}

    void addObstacle(ObstacleT obstacle) {obstacles_.push_back(obstacle);}
    void setBiasedIterations(size_t biased_iter) {biased_iter_ = biased_iter;}
    void setBiasSamplingRadius(T bias_sampling_radius) {bias_sampling_radius_ = bias_sampling_radius;}
    void setDistribution(PointDistributionT point_distribution) {point_distribution_ = point_distribution;}
    void setGoalRangeRadius(T goal_range_radius) {goal_range_radius_ = goal_range_radius;}
    void setMaximumIterations(size_t max_iter) {max_iter_ = max_iter;}
    void setMetric(Metric metric) {metric_ = metric;}
    void setNeighbourRadius(T neighbour_radius) {neighbour_radius_ = neighbour_radius;}
    void setOptimizationIterations(size_t optimization_iter) {optimization_iter_ = optimization_iter;}
    void setPathCheckResolution(T path_check_resolution) {path_check_resolution_ = path_check_resolution;}
    void setRewiringRadius(T rewiring_radius) {rewiring_radius_ = rewiring_radius;}
    void setSystem(SystemT system) {system_ = system;}

    void getBiasedIterations(size_t& biased_iter) {biased_iter = biased_iter_;}
    void getBiasSamplingRadius(T& bias_sampling_radius) {bias_sampling_radius = bias_sampling_radius_;}
    void getGoal(NodeT*& goal) {goal = goal_;}
    void getGoalRangeRadius(T& goal_range_radius) {goal_range_radius = goal_range_radius_;}
    void getMetric(Metric& metric) {metric = metric_;}
    void getMaximumIterations(size_t& max_iter) {max_iter = max_iter_;}
    void getNeighbourRadius(T& neighbour_radius) {neighbour_radius = neighbour_radius_;}
    void getNodes(std::vector<NodeT*>& nodes) {nodes = nodes_;}
    void getObstacles(std::vector<ObstacleT>& obstacles) {obstacles = obstacles_;}
    void getOptimizationIterations(size_t& optimization_iter) {optimization_iter = optimization_iter_;}
    void getRewiringRadius(T& rewiring_radius) {rewiring_radius = rewiring_radius_;}
    void getPathCheckResolution(T& path_check_resolution) {path_check_resolution = path_check_resolution_;}
    void getSolution(std::vector<NodeT*>& solution);
    T getSolutionCost() {return goal_->cost;}
    void getStart(NodeT*& start) {start = start_;}


private:
    void createBiasedDistribution(NodeT*& point);
    bool doesPathLieInFreeSpace(NodeT*& point, NodeT*& new_point);
    bool doesPointLieInFreeSpace(NodeT*& point);
    bool findNearestNeighbor(NodeT*& point, NodeT*& neighbor);
    bool isGoalWithinRange(NodeT*& point) {return metric_(point, goal_) < goal_range_radius_;}
    void optimizePath();
    void optimizePathWithBiasedSampling();
    void optimizePathWithTriangleInequality();
    void projectRandomPointTowardsNearest(NodeT*& point, NodeT*& nearest_point) const;
    void rewiring(NodeT*& point);
    void sampleRandomPoint(NodeT*& point);
    void sampleRandomPointWithinBias(NodeT*& point);

    NodeT* start_;
    NodeT* goal_;
    SystemT system_;  // agent/robot/...

    Metric metric_ {euclideanMetric};
    PointDistributionT point_distribution_;
    PointDistributionT biased_point_distribution_;

    size_t max_iter_ {1000};
    size_t biased_iter_ {50};
    size_t optimization_iter_ {5};
    T goal_range_radius_ {0.8};
    T neighbour_radius_ {0.8};
    T rewiring_radius_ {0.5};
    T path_check_resolution_ {0.01};
    T bias_sampling_radius_ {0.9};
    std::vector<NodeT*> nodes_;
    std::vector<ObstacleT> obstacles_;
    bool is_solution_found_ {false};
};


// RDT(NodeT* start, Node* goal)
template <typename T, size_t dim>
RDT<T, dim>::RDT(NodeT* start, NodeT* goal) {
    start_ = start;
    goal_ = goal;
}

// void sampleRandomPoint(NodeT*& point)
template <typename T, size_t dim>
void RDT<T, dim>::sampleRandomPoint(NodeT*& point) {
    point_distribution_.sample(*point);
}

// void findNearestNeighbor(NodeT*& point, NodeT*& neighbor)
template <typename T, size_t dim>
bool RDT<T, dim>::findNearestNeighbor(NodeT*& point, NodeT*& neighbor) {

    auto it = std::min_element(
            nodes_.begin(), nodes_.end(),
            [point, this](auto &left, auto &right) {
                return metric_(point, left) < metric_(point, right);
            }
    );

    if (it != nodes_.end()) {
        neighbor = *it;
        return true;
    }

    return false;
}

// void projectRandomPointTowardsNearest(NodeT*& point, NodeT*& nearest_point)
template<typename T, size_t dim>
void RDT<T, dim>::projectRandomPointTowardsNearest(NodeT*& point, NodeT*& nearest_point) const {
    T dist = metric_(point, nearest_point);
    if (dist > neighbour_radius_) {
        T rho = neighbour_radius_ / dist;

        for (int i=0; i<dim; i++) {
            point->operator[](i) = (1 - rho) * nearest_point->operator[](i) + rho * point->operator[](i);
        }
    }
}

// void rewiring(NodeT*& point)
template<typename T, size_t dim>
void RDT<T, dim>::rewiring(NodeT*& point) {
    T dist = 0.0;

    // rewire current node within the ball of radius r
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
        dist = metric_(point, *it);
        if (dist < rewiring_radius_) {
            if (point->cost > (*it)->cost + dist) {
                point->parent = *it;
                point->cost = (*it)->cost + dist;
            }
        }
    }

    // rewire the other nodes within the ball of radius r
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
        dist = metric_(point, *it);
        if (dist < rewiring_radius_ && *it != point->parent) {
            if ((*it)->cost > point->cost + dist) {
                (*it)->parent = point;
                (*it)->cost = point->cost + dist;
            }
        }
    }
}

// bool doesPointLieInFreeSpace(NodeT*& point)
template<typename T, size_t dim>
bool RDT<T, dim>::doesPointLieInFreeSpace(NodeT*& point) {
    for (auto &obstacle: obstacles_) {
        if (obstacle.isPointIncluded(*point))
            return false;

        system_.moveToPoint(*point);
        if (areObjectsColliding(system_, obstacle))
            return false;
    }
    return true;
}

// bool doesPathLieInFreeSpace(NodeT*& point1, NodeT*& point2)
template<typename T, size_t dim>
bool RDT<T, dim>::doesPathLieInFreeSpace(NodeT*& point, NodeT*& new_point) {
    T dist = metric_(point, new_point);
    size_t N = dist / path_check_resolution_;
    T delta = path_check_resolution_ / dist;
    NodeT* n = new NodeT;
    for (size_t i=1; i<N; i++) {
        *n = (1 - delta * i) * (*point) + delta * i * (*new_point);
        if (!doesPointLieInFreeSpace(n))
            return false;
    }
    return true;
}

// bool run()
template <typename T, size_t dim>
void RDT<T, dim>::run() {
    nodes_.push_back(start_);
    size_t iter = 0;
    while (iter < max_iter_) {
        NodeT* point = new NodeT;
        sampleRandomPoint(point);
        NodeT* nearest_neighbor = new NodeT;
        if (findNearestNeighbor(point, nearest_neighbor)) {
            projectRandomPointTowardsNearest(point, nearest_neighbor);

            // check if point and nearest_neighbor are in free space
            // as well as the path between them
            if (!doesPointLieInFreeSpace(point)
                    || !doesPathLieInFreeSpace(point, nearest_neighbor))
                continue;

            // update cost
            point->parent = nearest_neighbor;
            T cost_metric = metric_(point, nearest_neighbor);
            point->cost = nearest_neighbor->cost + cost_metric;
            // rewiring nodes close to point
            rewiring(point);
            nodes_.push_back(point);

            if (isGoalWithinRange(point)) {
                goal_->parent = point;
                goal_->cost = point->cost + metric_(point, goal_);
                printf("Goal found in %zu iterations\n", iter);
                optimizePath();
                is_solution_found_ = true;
                return;
            }
        }
        iter++;
    }
}

// void optimizePathWithTriangleInequality()
template<typename T, size_t dim>
void RDT<T, dim>::optimizePathWithTriangleInequality() {
    NodeT* node = goal_;
    NodeT* n1 = new NodeT;
    NodeT* n2 = new NodeT;
    std::vector<NodeT*> path;
    path.push_back(node);
    while (node != nullptr) {
        n1 = node->parent;
        if (n1 == nullptr)
            break;
        n2 = n1->parent;
        if (n2 == nullptr)
            break;

        if (doesPathLieInFreeSpace(node, n2)) {

            if (n2->parent != nullptr && !doesPathLieInFreeSpace(n2->parent, n2)) {
                    node = n1;
                    continue;
            }

            node->parent = node->parent->parent;
            node->cost = n2->cost + metric_(node, n2);
            continue;
        }
        node = n1;

        path.push_back(node);
    }
    path.push_back(start_);

    // reverse path
    std::reverse(path.begin(), path.end());
    // recomputing cost
    for (size_t i=1; i<path.size(); i++) {
        path[i]->cost = path[i-1]->cost + metric_(path[i], path[i-1]);
    }

}

// void createBiasedDistribution(NodeT*& point)
template <typename T, size_t dim>
void RDT<T, dim>::createBiasedDistribution(NodeT*& point) {
    std::vector<std::pair<T, T>> intervals;
    intervals.reserve(dim);
    for (int i=0; i<dim; i++) {
        intervals.emplace_back(
                point->operator[](i) - bias_sampling_radius_,
                point->operator[](i) + bias_sampling_radius_
                );
    }
    biased_point_distribution_ = PointDistributionT(intervals);
}

// void sampleRandomPointWithinBias(NodeT*& point)
template<typename T, size_t dim>
void RDT<T, dim>::sampleRandomPointWithinBias(NodeT*& point) {
    biased_point_distribution_.sample(*point);
}

// void optimizePathWithBiasedSampling()
template<typename T, size_t dim>
void RDT<T, dim>::optimizePathWithBiasedSampling() {
    NodeT* node = goal_;
    while (node != nullptr) {
        NodeT* n1 = node->parent;
        if (n1 == nullptr)
            break;
        // ensure the start node is not modified
        if (n1->parent == nullptr)
            break;

        T min_cost = metric_(n1->parent, n1) + metric_(n1, node);
        createBiasedDistribution(n1);
        // printf("Biased Iter: %zu\n", biased_iter_);

        for (size_t i=0; i<biased_iter_; i++) {
            NodeT* n2 = new NodeT;
            sampleRandomPointWithinBias(n2);
            T cost = metric_(n1->parent, n2) + metric_(n2, node);
            if (cost < min_cost) {
                if ((doesPointLieInFreeSpace(n2) && doesPathLieInFreeSpace(n2, node) && doesPathLieInFreeSpace(n2, n1->parent))) {
                    min_cost = cost;
                    n2->parent = n1->parent;
                    n2->cost = cost;
                    node->cost = metric_(n2, node) + n2->cost;
                    node->parent = n2;
                    nodes_.push_back(n2);
                }
            }
        }
        node = n1;
    }
}

// void optimizePath()
template<typename T, size_t dim>
void RDT<T, dim>::optimizePath() {
    for (size_t i=0; i<optimization_iter_; i++) {
        T cost = getSolutionCost();
        optimizePathWithTriangleInequality();
        optimizePathWithBiasedSampling();
        T new_cost = getSolutionCost();
        if (fabs(new_cost - cost) < 1e-6) {
            break;
        }
    }
}

// void getSolution()
template<typename T, size_t dim>
void RDT<T, dim>::getSolution(std::vector<NodeT*>& solution) {
    solution.clear();
    NodeT* node = goal_;
    while (node != nullptr) {
        solution.push_back(node);
        node = node->parent;
    }
    std::reverse(solution.begin(), solution.end());
}

#endif //PLANNING_ALGORITHMS_CPP_RDT_HPP


