#include <iostream>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include "Digraph.h"


Digraph::Digraph(const std::vector<std::vector<int>> &adj_list) {
    num_forests = 0;
    generator = std::mt19937(239);

    adj_list_ = std::vector<std::vector<std::shared_ptr<DirectedEdge>>>(adj_list.size());
    adj_list_inv_ = std::vector<std::vector<std::shared_ptr<DirectedEdge>>>(adj_list.size());
    adj_list_full_ = std::vector<std::vector<std::shared_ptr<DirectedEdge>>>(adj_list.size());
    for (int tail = 0; tail < static_cast<int>(adj_list.size()); ++tail) {
        for (int head: adj_list[tail]) {
            DirectedEdge new_edge(head, tail);
            std::shared_ptr<DirectedEdge> edge_ptr = std::make_shared<DirectedEdge>(new_edge);
            adj_list_[tail].push_back(edge_ptr);
            adj_list_inv_[head].push_back(edge_ptr);
            adj_list_full_[tail].push_back(edge_ptr);
            adj_list_full_[head].push_back(edge_ptr);
        }
    }
}

void Digraph::PrintGraph() {
    std::cout << "Adjacency list:\n";
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        std::cout << vertex << ":";
        for (auto &edge: adj_list_[vertex]) {
            std::cout << " (head: " << edge->head << ", forest: " << edge->forest << ")";
        }
        std::cout << "\n";
    }
}

int Digraph::EdgeIsJoining(const std::shared_ptr<DirectedEdge> &edge) {
    // if the edge is joining, returns the minimal index of a suitable forest
    // else returns -1

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        if (edge->forest == forest_index) {
            continue;
        }
        if (disjoint_components[forest_index].Representative(edge->head) !=
            disjoint_components[forest_index].Representative(edge->tail)) {
            return forest_index;
        }
    }

    return -1;
}

void Digraph::GenerateForests(int k) {
    for (int i = 0; i < k; ++i) {
        GenerateNextForest();
    }
}

void Digraph::GenerateNextForest() {
    ++num_forests;
    DisjointSets next_sets(static_cast<int>(adj_list_.size()));
    disjoint_components.push_back(next_sets);

    std::vector<int> vertices;
    vertices.reserve(static_cast<int>(adj_list_.size()));
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        vertices.push_back(vertex);
    }
    std::shuffle(vertices.begin(), vertices.end(), generator);

    for (int vertex : vertices) {
        auto path = AugmentingPath(vertex);
        while (!path.empty()) {
            // --------------------------------hypothesis testing
            if ((Deficit(vertex) > 1) && (EdgeIsJoining(path.back()) != num_forests - 1)) {
                std::cout << "example found path to the outside" << std::endl;
                PrintGraph();
                std::cout << "path:" << std::endl;
                for (const auto& edge : path) {
                    std::cout << edge->tail << " " << edge->head << std::endl;
                }
            }
            // --------------------------------hypothesis testing

            AugmentPath(path);
            path = AugmentingPath(vertex);
        }
    }
}

std::vector<std::vector<std::pair<int, int>>> Digraph::GetForests() {
    // returns a vector of forests, each forest represented with an edge list
    std::vector<std::vector<std::pair<int, int>>> ans(num_forests, std::vector<std::pair<int, int>>(0));

    for (auto &out_edges: adj_list_) {
        for (const auto &edge: out_edges) {
            if (edge->forest != -1) {
                ans[edge->forest].emplace_back(edge->head, edge->tail);
            }
        }
    }

    return ans;
}

void Digraph::AugmentPath(const std::vector<std::shared_ptr<DirectedEdge>> &path) {
    // std::cout << "AugmentPath" << std::endl;

    int final_color = EdgeIsJoining(path.back());
    if (final_color == -1) {
        throw std::runtime_error("augmenting path not ending in a joining edge");
    }
    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
        path[i]->forest = path[i + 1]->forest;
    }
    path.back()->forest = final_color;

    disjoint_components[final_color].Unite(path.back()->head, path.back()->tail);
}

std::vector<std::shared_ptr<DirectedEdge>>
Digraph::FundamentalCycle(const std::shared_ptr<DirectedEdge> &edge, int forest_index) {
    // returns all edges in a fundamental cycle in a given forest of the given edge
    // TODO this is O(m), rewrite

    // std::cout << "FundamentalCycle" << std::endl;

    if (edge->forest == forest_index) {
        return {};
    }

    int initial_vertex = edge->head;
    int final_vertex = edge->tail;

    if (disjoint_components[forest_index].Representative(initial_vertex) !=
        disjoint_components[forest_index].Representative(final_vertex)) {
        // edge is joining for this forest
        return {};
    }

    std::queue<int> queue;
    queue.push(initial_vertex);

    std::vector<bool> visited_vertices(adj_list_.size(), false);
    std::vector<std::shared_ptr<DirectedEdge>> edge_to_parent(adj_list_.size(), edge);
    visited_vertices[initial_vertex] = true;

    bool path_exists = false;
    while ((!queue.empty()) && (!path_exists)) {
        int current_vertex = queue.front();
        queue.pop();

        for (const auto &next_edge: adj_list_full_[current_vertex]) {
            int next_vertex = next_edge->AnotherVertex(current_vertex);

            if ((next_edge->forest != forest_index) || (visited_vertices[next_vertex])) {
                continue;
            }

            queue.push(next_vertex);
            edge_to_parent[next_vertex] = next_edge;
            visited_vertices[next_vertex] = true;
            if (next_vertex == final_vertex) {
                path_exists = true;
                break;
            }
        }
    }

    if (!path_exists) {
        throw std::runtime_error("fundamental cycle not found, but edge was not joining");
    }

    std::vector<std::shared_ptr<DirectedEdge>> fundamental_cycle;
    int path_vertex = final_vertex;
    while (path_vertex != initial_vertex) {
        fundamental_cycle.push_back(edge_to_parent[path_vertex]);
        path_vertex = edge_to_parent[path_vertex]->AnotherVertex(path_vertex);
    }
    return fundamental_cycle;
}

std::vector<std::shared_ptr<DirectedEdge>> Digraph::Neighbors(const std::shared_ptr<DirectedEdge> &edge) {
    // std::cout << "Neighbors" << std::endl;

    std::vector<std::shared_ptr<DirectedEdge>> neighbors;

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        auto fundamental_cycle = FundamentalCycle(edge, forest_index);
        for (const auto &neighbor: fundamental_cycle) {
            neighbors.push_back(neighbor);
        }
    }

    if (edge->forest != -1) {
        for (const auto &same_head_edge: adj_list_inv_[edge->head]) {
            if (same_head_edge->forest == -1) {
                neighbors.push_back(same_head_edge);
            }
        }
    }

    return neighbors;
}

int Digraph::Indeg(int vertex) {
    int indeg = 0;
    for (const auto &same_head_edge: adj_list_inv_[vertex]) {
        if (same_head_edge->forest != -1) {
            ++indeg;
        }
    }
    return indeg;
}

int Digraph::Deficit(int vertex) {
    int indeg_forests = Indeg(vertex);
    int indeg = static_cast<int>(adj_list_inv_[vertex].size());

    if (indeg >= num_forests) {
        return num_forests - indeg_forests;
    }
    return indeg - indeg_forests;
}

std::vector<std::shared_ptr<DirectedEdge>> Digraph::AugmentingPath(int vertex) {
    // returns the shortest augmenting path starting from vertex, if the path exists, else {}

    // std::cout << "AugmentingPath" << std::endl;

    if (Deficit(vertex) == 0) {
        return {};
    }

    std::queue<std::shared_ptr<DirectedEdge>> queue;
    std::unordered_set<std::shared_ptr<DirectedEdge>> visited;
    for (const auto &edge: adj_list_inv_[vertex]) {
        if (edge->forest == -1) {
            if (EdgeIsJoining(edge) != -1) {
                // path of length 1
                return {edge};
            }
            queue.push(edge);
            visited.insert(edge);
        }
    }

    std::unordered_map<std::shared_ptr<DirectedEdge>, std::shared_ptr<DirectedEdge>> parents;

    while (!queue.empty()) {
        auto current_edge = queue.front();
        queue.pop();

        auto neighbors = Neighbors(current_edge);
        for (auto neighbor: neighbors) {
            if (visited.find(neighbor) != visited.end()) {
                continue;
            }

            visited.insert(neighbor);
            queue.push(neighbor);
            parents[neighbor] = current_edge;

            int final_forest_index = EdgeIsJoining(neighbor);

            if (final_forest_index != -1) {
                // found an augmenting path
                std::vector<std::shared_ptr<DirectedEdge>> path;
                while (parents.find(neighbor) != parents.end()) {
                    path.push_back(neighbor);
                    neighbor = parents[neighbor];
                }
                path.push_back(neighbor);
                std::reverse(path.begin(), path.end());
                return path;
            }
        }
    }

    return {};
}

std::vector<int> Digraph::ListOfComponentDeficits() {
    std::vector<int> deficits(adj_list_.size(), 0);
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        deficits[disjoint_components[num_forests - 1].Representative(vertex)] += Deficit(vertex);
    }
    return deficits;
}


