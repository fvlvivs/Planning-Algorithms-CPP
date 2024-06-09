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


#ifndef XI_PLANNING_ALGORITHMS_DISCRETESEARCH_HPP
#define XI_PLANNING_ALGORITHMS_DISCRETESEARCH_HPP

#include <type_traits>

#include "utils.hpp"
#include "Core.hpp"
#include "SeparatingAxis.hpp"
#include "Polyhedra.hpp"


template <typename T, size_t dim>
class DiscreteSearch {
    typedef Node<T, dim> NodeT;
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
    DiscreteSearch(NodeT* start, NodeT* goal);
    ~DiscreteSearch() = default;

    void addObstacle(ObstacleT obstacle) {obstacles_.push_back(obstacle);}
    void setMaximumIterations(size_t max_iter) {max_iter_ = max_iter;}
    void setMetric(Metric metric) {metric_ = metric;}
    void setSpaceLimits(std::vector<std::pair<T, T>> limits);
    void setStepSize(T step_size) {step_size_ = step_size;}
    void setSystem(SystemT system) {system_ = system;}

    void getGoal(NodeT*& goal) {goal = goal_;}
    void getMaximumIterations(size_t& max_iter) {max_iter = max_iter_;}
    void getMetric(Metric& metric) {metric = metric_;}
    void getObstacles(std::vector<ObstacleT>& obstacles) {obstacles = obstacles_;}
    void getSolution(std::vector<NodeT*>& solution);
    void getSpaceLimits(std::vector<std::pair<T, T>>& limits) {limits = limits_;}
    T getStepSize() const {return step_size_;}
    void getSystem(SystemT& system) {system = system_;}

    bool isSolutionFound() const {return is_solution_found_;}
    void run();

protected:
    bool isGoalWithinTolerance(NodeT* node) const;
    bool isNodeInPriorityQueue(NodeT* node);
    bool isNodeAlreadyExplored(NodeT* node);
    bool isNodeInSpaceContained(NodeT* node) const;
    bool isSystemInCollision(NodeT* node);
    bool getNodeFromPriorityQueue(NodeT* node);
    void getActions(std::vector<NodeT*>& actions);
    void get2DActions(std::vector<NodeT*>& actions);
    void get3DActions(std::vector<NodeT*>& actions);
    void expandNode(NodeT* node);
    void getUniqueHash(NodeT* node, size_t& id);

    virtual void orderPriorityQueue() = 0;
    virtual void updateNewNodeCost(NodeT* node, NodeT* new_node) = 0;
    virtual void assignMaximumCost(NodeT* node) = 0;

    NodeT *start_;
    NodeT *goal_;

    size_t max_iter_ {1000};
    T step_size_ {0.5};
    T goal_tol_ {0.1};
    bool is_solution_found_ {false};
    Metric metric_ {euclideanMetric};
    SystemT system_;
    std::vector<std::pair<T, T>> limits_;
    std::vector<NodeT*> priority_queue_;
    std::vector<NodeT*> visited_nodes_;
    std::vector<ObstacleT> obstacles_;

};

// DiscreteSearch(NodeT* start, NodeT* goal)
template<typename T, size_t dim>
DiscreteSearch<T, dim>::DiscreteSearch(NodeT *start, NodeT *goal) : start_(start), goal_(goal) {
}

// void setSpaceLimits(std::vector<std::pair<T, T>> limits)
template<typename T, size_t dim>
void DiscreteSearch<T, dim>::setSpaceLimits(std::vector<std::pair<T, T>> limits) {
    limits_ = limits;
}

// bool isGoalWithinTolerance(NodeT* node) const
template<typename T, size_t dim>
bool DiscreteSearch<T, dim>::isGoalWithinTolerance(NodeT *node) const {
    return metric_(node, goal_) < goal_tol_;
}

// bool isNodeInPriorityQueue(NodeT* node)
template<typename T, size_t dim>
bool DiscreteSearch<T, dim>::isNodeInPriorityQueue(NodeT* node) {
    auto it = std::find_if(
            priority_queue_.begin(),
            priority_queue_.end(),
            [&node](NodeT* n) {
                return *n == node;
            });

    return it != priority_queue_.end();
}

// bool isNodeAlreadyExplored(NodeT* node)
template<typename T, size_t dim>
bool DiscreteSearch<T, dim>::isNodeAlreadyExplored(NodeT* node) {
    auto it = std::find_if(
            visited_nodes_.begin(),
            visited_nodes_.end(),
            [node](const NodeT* n) {
                return *n == node;
            });

    return it != visited_nodes_.end();
}

// bool isNodeInSpaceContained(NodeT* node)
template<typename T, size_t dim>
bool DiscreteSearch<T, dim>::isNodeInSpaceContained(NodeT* node) const {
    for (size_t i = 0; i < dim; i++) {
        if (node->operator[](i) < limits_[i].first || node->operator[](i) > limits_[i].second)
            return false;
    }
    return true;
}

// bool getNodeFromPriorityQueue(NodeT* node)
template<typename T, size_t dim>
bool DiscreteSearch<T, dim>::getNodeFromPriorityQueue(NodeT* node) {
    auto it = std::find_if(
            priority_queue_.begin(),
            priority_queue_.end(),
            [&node](NodeT* n) {
                return *n == node;
            });

    if (it != priority_queue_.end()) {
        node = *it;
        return true;
    }
    return false;
}

// bool isSystemInCollision(NodeT* node)
template<typename T, size_t dim>
bool DiscreteSearch<T, dim>::isSystemInCollision(NodeT* node) {
    system_.moveToPoint(*node);
    for (auto& obstacle: obstacles_) {
        if (areObjectsColliding(system_, obstacle))
            return true;
    }
    return false;
}

// void getActions(std::vector<NodeT*>& actions)
template<typename T, size_t dim>
void DiscreteSearch<T, dim>::getActions(std::vector<NodeT*>& actions) {
    if constexpr (dim == 2) {
        get2DActions(actions);
    } else {
        get3DActions(actions);
    }
}

// void get2DActions(std::vector<NodeT*>& actions)
template<typename T, size_t dim>
void DiscreteSearch<T, dim>::get2DActions(std::vector<NodeT*>& actions) {
    actions.emplace_back(new NodeT(0, step_size_));
    actions.emplace_back(new NodeT(0, -step_size_));
    actions.emplace_back(new NodeT(step_size_, 0));
    actions.emplace_back(new NodeT(-step_size_, 0));
}

// void get3DActions(std::vector<NodeT*>& actions)
template<typename T, size_t dim>
void DiscreteSearch<T, dim>::get3DActions(std::vector<NodeT*>& actions) {
    actions.emplace_back(new NodeT(0, 0, step_size_));
    actions.emplace_back(new NodeT(0, 0, -step_size_));
    actions.emplace_back(new NodeT(0, step_size_, 0));
    actions.emplace_back(new NodeT(0, -step_size_, 0));
    actions.emplace_back(new NodeT(step_size_, 0, 0));
    actions.emplace_back(new NodeT(-step_size_, 0, 0));
}

// void getUniqueHash(NodeT* node, size_t& id)
template <typename T, size_t dim>
void DiscreteSearch<T, dim>::getUniqueHash(DiscreteSearch::NodeT *node, size_t &id) {
    // FNV-1a hash type
    id = 2166136261;
    id = (id ^ (size_t(node->x() * 2654435769)) << 16);
    id = (id ^ (size_t(node->y() * 2654435769)) << 8);
    if (dim == 3)
        id = (id ^ (size_t(node->z() * 2654435769)));
}

// void expandNode(NodeT* node)
template<typename T, size_t dim>
void DiscreteSearch<T, dim>::expandNode(DiscreteSearch::NodeT *node) {
    std::vector<NodeT*> actions;
    getActions(actions);

    for (auto &action : actions) {
        NodeT* new_node = new NodeT;
        *new_node = *node + *action;
        T cost = node->cost_to_come + metric_(node, new_node);

        if (isNodeAlreadyExplored(new_node))
            continue;

        if (!isNodeInSpaceContained(new_node)) {
            assignMaximumCost(new_node);
            visited_nodes_.push_back(new_node);
            continue;
        }

        if (isSystemInCollision(new_node)) {
            assignMaximumCost(new_node);
            visited_nodes_.push_back(new_node);
            continue;
        }

        new_node->parent = node;

        if (!isNodeInPriorityQueue(new_node)) {
            updateNewNodeCost(node, new_node);
            priority_queue_.push_back(new_node);
        } else {
            if (getNodeFromPriorityQueue(new_node)) {
                if (new_node->cost_to_come > cost) {
                    new_node->parent = node;
                    updateNewNodeCost(node, new_node);
                }
            }
        }
    }
    visited_nodes_.push_back(node);
}

// void run()
template <typename T, size_t dim>
void DiscreteSearch<T, dim>::run() {
    size_t i = 0;
    priority_queue_.push_back(start_);

    while (!priority_queue_.empty() && i < max_iter_) {
        orderPriorityQueue();
        NodeT* node = priority_queue_.front();
        priority_queue_.erase(priority_queue_.begin());
        if (isGoalWithinTolerance(node)) {
            goal_->cost_to_come = node->cost_to_come;
            goal_->parent = node->parent;
            is_solution_found_ = true;
            printf("Solution found in %zu iterations\n", i);
            break;
        }
        expandNode(node);
        i++;
    }

    if (i == max_iter_)
        printf("Maximal number of iterations reached\n");
}

// void getSolution(std::vector<NodeT*>& solution)
template<typename T, size_t dim>
void DiscreteSearch<T, dim>::getSolution(std::vector<NodeT*>& solution) {
    solution.clear();
    NodeT* node = goal_;
    while (node != nullptr) {
        solution.push_back(node);
        node = node->parent;
    }
    std::reverse(solution.begin(), solution.end());
}

#endif //XI_PLANNING_ALGORITHMS_DISCRETESEARCH_HPP
