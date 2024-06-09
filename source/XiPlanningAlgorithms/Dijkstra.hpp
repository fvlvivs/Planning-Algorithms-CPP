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


#ifndef XI_PLANNING_ALGORITHMS_DIJKSTRA_HPP
#define XI_PLANNING_ALGORITHMS_DIJKSTRA_HPP

#include "Core.hpp"
#include "DiscreteSearch.hpp"

template <typename T, size_t dim>
class Dijkstra : public DiscreteSearch<T, dim> {
    typedef Node<T, dim> NodeT;

public:
    Dijkstra(NodeT* start, NodeT* goal) : DiscreteSearch<T, dim>(start, goal) {}
    ~Dijkstra() = default;

private:
    void updateNewNodeCost(NodeT* node, NodeT* new_node);
    void assignMaximumCost(NodeT* node);
};

// void updateNewNodeCost(NodeT* node, NodeT* new_node)
template <typename T, size_t dim>
void Dijkstra<T, dim>::updateNewNodeCost(NodeT* node, NodeT* new_node) {
    new_node->cost_to_come = node->cost_to_come +
            this->metric_(node, new_node);
}

// void assignMaximumCost(NodeT* node)
template <typename T, size_t dim>
void Dijkstra<T, dim>::assignMaximumCost(NodeT* node) {
    node->cost_to_come = std::numeric_limits<T>::max();
}

#endif //XI_PLANNING_ALGORITHMS_DIJKSTRA_HPP
