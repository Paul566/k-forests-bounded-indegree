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
    num_edges = 0;

    adj_list_ = std::vector<std::vector<std::shared_ptr<DirectedEdge>>>(adj_list.size());
    adj_list_inv_ = std::vector<std::vector<std::shared_ptr<DirectedEdge>>>(adj_list.size());
    adj_list_full_ = std::vector<std::vector<std::shared_ptr<DirectedEdge>>>(adj_list.size());
    for (int tail = 0; tail < static_cast<int>(adj_list.size()); ++tail) {
        for (int head : adj_list[tail]) {
            ++num_edges;
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
        for (auto &edge : adj_list_[vertex]) {
            std::cout << " (head: " << edge->head << ", forest: " << edge->forest << ")";
        }
        std::cout << "\n";
    }
}

int Digraph::EdgeIsJoining(const std::shared_ptr<DirectedEdge> &edge) {
    // if the edge is joining, returns the minimal index of a suitable forest
    // else returns -1
    // TODO do it with binary search

    if (disjoint_components[num_forests - 1].Representative(edge->head) ==
        disjoint_components[num_forests - 1].Representative(edge->tail)) {
        // not joining for last forest => not joining for any forest
        return -1;
    }

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        if (edge->forest == forest_index) {
            continue;
        }
        if (disjoint_components[forest_index].Representative(edge->head) !=
            disjoint_components[forest_index].Representative(edge->tail)) {
            return forest_index;
        }
    }

    throw std::runtime_error("in EdgeIsJoining: edge joining for the last forest but not joining for any forest");
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

    std::vector<int> new_depth(static_cast<int>(adj_list_.size()), 0);
    depth.push_back(new_depth);

    std::vector<std::shared_ptr<DirectedEdge>> new_parents(static_cast<int>(adj_list_.size()), nullptr);
    edge_to_parent.push_back(new_parents);

    vertex_active = std::vector<bool>(static_cast<int>(adj_list_.size()), true);

    auto vertices = ActiveVerticesPerComponent();
//    std::cout << "deficient vertices per component:" << std::endl;
//    for (int v : vertices) {
//        std::cout << v << std::endl;
//    }
    int total_deficit = TotalModifiedDeficit();
    while (!vertices.empty()) {
        bool success = Epoch(2 * num_edges * num_forests / total_deficit + 1);
        vertices = ActiveVerticesPerComponent();
        total_deficit = TotalModifiedDeficit();
//        PrintGraph();
//        std::cout << "deficient vertices per component:" << std::endl;
//        for (int v : vertices) {
//            std::cout << v << std::endl;
//        }
    }
}

std::vector<std::vector<std::pair<int, int>>> Digraph::GetForests() {
    // returns a vector of forests, each forest represented with an edge list
    std::vector<std::vector<std::pair<int, int>>> ans(num_forests, std::vector<std::pair<int, int>>(0));

    for (auto &out_edges : adj_list_) {
        for (const auto &edge : out_edges) {
            if (edge->forest != -1) {
                ans[edge->forest].emplace_back(edge->head, edge->tail);
            }
        }
    }

    return ans;
}

void Digraph::AugmentPath(const std::vector<std::shared_ptr<DirectedEdge>> &path) {
    // augments a path

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
Digraph::FundamentalCycleLegacy(const std::shared_ptr<DirectedEdge> &edge, int forest_index) {
    // returns all edges in a fundamental cycle in a given forest of the given edge
    // this is O(m), use only for testing

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
    std::vector<std::shared_ptr<DirectedEdge>> backwards_edge(adj_list_.size(), edge);
    visited_vertices[initial_vertex] = true;

    bool path_exists = false;
    while ((!queue.empty()) && (!path_exists)) {
        int current_vertex = queue.front();
        queue.pop();

        for (const auto &next_edge : adj_list_full_[current_vertex]) {
            int next_vertex = next_edge->AnotherVertex(current_vertex);

            if ((next_edge->forest != forest_index) || (visited_vertices[next_vertex])) {
                continue;
            }

            queue.push(next_vertex);
            backwards_edge[next_vertex] = next_edge;
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
        fundamental_cycle.push_back(backwards_edge[path_vertex]);
        path_vertex = backwards_edge[path_vertex]->AnotherVertex(path_vertex);
    }
    return fundamental_cycle;
}

std::vector<std::shared_ptr<DirectedEdge>> Digraph::NeighborsLegacy(const std::shared_ptr<DirectedEdge> &edge) {
    // std::cout << "Neighbors" << std::endl;

    std::vector<std::shared_ptr<DirectedEdge>> neighbors;

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        auto fundamental_cycle = FundamentalCycleLegacy(edge, forest_index);
        for (const auto &neighbor : fundamental_cycle) {
            neighbors.push_back(neighbor);
        }
    }

    if (edge->forest != -1) {
        for (const auto &same_head_edge : adj_list_inv_[edge->head]) {
            if (same_head_edge->forest == -1) {
                neighbors.push_back(same_head_edge);
            }
        }
    }

    return neighbors;
}

int Digraph::Indeg(int vertex) {
    int indeg = 0;
    for (const auto &same_head_edge : adj_list_inv_[vertex]) {
        if (same_head_edge->forest != -1) {
            ++indeg;
        }
    }
    return indeg;
}

int Digraph::Deficit(int vertex) {
    int indeg_forests = Indeg(vertex);
    return num_forests - indeg_forests;
}

std::vector<std::shared_ptr<DirectedEdge>> Digraph::AugmentingPathLegacy(int vertex) {
    // returns the shortest augmenting path starting from vertex, if the path exists, else {}

    // std::cout << "AugmentingPath" << std::endl;

    if (Deficit(vertex) == 0) {
        return {};
    }

    std::queue<std::shared_ptr<DirectedEdge>> queue;
    std::unordered_set<std::shared_ptr<DirectedEdge>> visited;
    for (const auto &edge : adj_list_inv_[vertex]) {
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

        auto neighbors = NeighborsLegacy(current_edge);
        for (auto neighbor : neighbors) {
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

void Digraph::Validate() {
    // checks that the current solution is optimal, throws error if it's not
    // runs slowly, should only be used for stress testing purposes

    std::vector<int> vertices;
    vertices.reserve(static_cast<int>(adj_list_.size()));
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        vertices.push_back(vertex);
    }
    std::shuffle(vertices.begin(), vertices.end(), generator);

    for (int vertex : vertices) {
        auto path = AugmentingPathLegacy(vertex);
        if (!path.empty()) {
            PrintGraph();

            std::cout << "found path:" << std::endl;
            for (const auto &edge : path) {
                std::cout << edge->tail << " " << edge->head << std::endl;
            }

            throw std::runtime_error("validation failed: found augmenting path");
        }
    }
}

std::vector<std::shared_ptr<DirectedEdge>> Digraph::Search(int vertex, int bound) {
    // returns an augmenting path if it founds one
    // aborts if more than bound edges labelled
    // also updates active/inactive labels

    if (Indeg(vertex) == num_forests) {
        vertex_active[vertex] = false;
        return {};
    }

    int edges_labelled = 0;
    std::queue<std::shared_ptr<DirectedEdge>> queue;
    for (const auto &edge : adj_list_inv_[vertex]) {
        if (edge->forest == -1) {
            if (EdgeIsJoining(edge) != -1) {
                return {edge};
            }
            queue.push(edge);
            ++edges_labelled;
        }
    }

    std::vector<std::unordered_set<int>> labelled_vertices(num_forests, {vertex});
    std::vector<int> labelled_roots(num_forests, vertex);
    // TODO move it to the level of Epoch and make it a vector of bools, it will probably be faster

    std::unordered_map<std::shared_ptr<DirectedEdge>, std::shared_ptr<DirectedEdge>> parents;
    // TODO maybe make it a label on Edge itself

    int forest_index = 0;
    std::shared_ptr<DirectedEdge> final_edge = nullptr;
    while ((!queue.empty()) && (edges_labelled <= bound)) {
        auto current_edge = queue.front();
        queue.pop();

        if (forest_index == current_edge->forest) {
            forest_index = (forest_index + 1) % num_forests;
        }

        if ((std::find(labelled_vertices[forest_index].begin(), labelled_vertices[forest_index].end(),
                       current_edge->head) != labelled_vertices[forest_index].end())
            && (std::find(labelled_vertices[forest_index].begin(), labelled_vertices[forest_index].end(),
                          current_edge->tail) != labelled_vertices[forest_index].end())) {
            // both ends are already in the subtree L_i
            // TODO maybe move in into UnlabelledFundamentalCycle
            continue;
        }

        auto cycle = UnlabelledFundamentalCycle(current_edge, forest_index, labelled_vertices, labelled_roots);
        if (cycle.empty()) {
            continue;
        }
        if (EdgeIsJoining(cycle[0]) != -1) {
            final_edge = cycle[0];
            parents[cycle[0]] = current_edge;
            // TODO get rid of this copy paste
            std::vector<std::shared_ptr<DirectedEdge>> path;
            while (parents.find(final_edge) != parents.end()) {
                path.push_back(final_edge);
                final_edge = parents[final_edge];
            }
            path.push_back(final_edge);
            std::reverse(path.begin(), path.end());

            if (path[0]->head != vertex) {
                throw std::runtime_error("in Search: head of the first edge is wrong");
            }
            return path;
        }
        // if there is a joining edge in cycle, cycle consists of one element, so there are no joining edges
        for (const auto &edge : cycle) {
            if (parents.find(edge) != parents.end()) {
                PrintGraph();
                throw std::runtime_error("some edge in UnlabelledFundamentalCycle was already labelled");
            }
            parents[edge] = current_edge;
            queue.push(edge);
            ++edges_labelled;

            for (const auto &next_edge : adj_list_inv_[edge->head]) {
                if (next_edge->forest != -1) {
                    continue;
                }
                if ((next_edge->head == vertex) || (parents.find(next_edge) != parents.end())) {
                    // been here, already labelled rho(next_edge->head)
                    break;
                }

                parents[next_edge] = edge;
                queue.push(next_edge);
                ++edges_labelled;

                if (EdgeIsJoining(next_edge) != -1) {
                    final_edge = next_edge;
                    // TODO get rid of this copy paste
                    std::vector<std::shared_ptr<DirectedEdge>> path;
                    while (parents.find(final_edge) != parents.end()) {
                        path.push_back(final_edge);
                        final_edge = parents[final_edge];
                    }
                    path.push_back(final_edge);
                    std::reverse(path.begin(), path.end());

                    if (path[0]->head != vertex) {
                        throw std::runtime_error("in Search: head of the first edge is wrong");
                    }
                    return path;
                }
            }
        }
    }

    if (final_edge == nullptr) {
        // std::cout << "making vertex " << vertex << " inactive" << std::endl;
        vertex_active[vertex] = false;
        return {};
    }

    throw std::runtime_error("in Search: final edge is not nullptr but haven't returned yet");
}

void Digraph::UpdateDepthsAndParents() {
    auto forest_adj_lists = FullAdjListPerForest();

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        std::vector<bool> visited(adj_list_.size(), false);
        for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
            if (visited[vertex]) {
                continue;
            }

            std::deque<int> stack;
            stack.push_back(vertex);
            depth[forest_index][vertex] = 0;
            edge_to_parent[forest_index][vertex] = nullptr;
            while (!stack.empty()) {
                int current_vertex = stack.back();
                stack.pop_back();
                for (const auto &next_edge : forest_adj_lists[forest_index][current_vertex]) {
                    if (next_edge != edge_to_parent[forest_index][current_vertex]) {
                        int child = next_edge->AnotherVertex(current_vertex);
                        stack.push_back(child);
                        depth[forest_index][child] = depth[forest_index][current_vertex] + 1;
                        edge_to_parent[forest_index][child] = next_edge;
                    }
                }
            }
        }
    }
}

int Digraph::TotalModifiedDeficit() {
    int answer = 0;
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        if (vertex_active[vertex]) {
            answer += Deficit(vertex);
        }
    }
    return answer;
}

std::vector<int> Digraph::ActiveVerticesPerComponent() {
    std::vector<int> answer;
    std::vector<bool> visited_components(adj_list_.size(), false);

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        if ((vertex_active[vertex]) &&
            (!visited_components[disjoint_components[num_forests - 1].Representative(vertex)])) {
            answer.push_back(vertex);
            visited_components[disjoint_components[num_forests - 1].Representative(vertex)] = true;
        }
    }

    return answer;
}

void Digraph::UpdateActivity() {
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        if (vertex_active[vertex]) {
            if (Deficit(vertex) == 0) {
                vertex_active[vertex] = false;
            }
        }
    }
}

bool Digraph::Epoch(int bound) {
    // bound: current bound for the Search
    // returns true if at least one augmentation was made

//    std::cout << "starting Epoch, num_forests: " << num_forests << std::endl;
//    PrintGraph();

    std::vector<bool> visited_components(adj_list_.size(), false);
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> paths;

    auto deficient_vertices = ActiveVerticesPerComponent();
    for (int vertex : deficient_vertices) {
//        std::cout << "Searching from vertex " << vertex << std::endl;

        if (visited_components[disjoint_components[num_forests - 1].Representative(vertex)]) {
            continue;
        }
        auto path = Search(vertex, bound);
        if (!path.empty()) {
            visited_components[disjoint_components[num_forests - 1].Representative(path.back()->head)] = true;
            visited_components[disjoint_components[num_forests - 1].Representative(path.back()->tail)] = true;
            paths.push_back(path);
        }
    }

    if (paths.empty()) {
        return false;
    }

    for (const auto &path : paths) {
        AugmentPath(path);
    }

    UpdateActivity();
    UpdateDepthsAndParents();

    return true;
}

std::vector<std::vector<std::vector<std::shared_ptr<DirectedEdge>>>> Digraph::FullAdjListPerForest() {
    // returns a collection of adjacency lists for each edge color
    std::vector<std::vector<std::vector<std::shared_ptr<DirectedEdge>>>> answer;
    answer.reserve(num_forests);
    for (int i = 0; i < num_forests; ++i) {
        answer.emplace_back(adj_list_.size(), std::vector<std::shared_ptr<DirectedEdge>>());
    }

    for (const auto &list : adj_list_) {
        for (const auto &edge : list) {
            if (edge->forest != -1) {
                answer[edge->forest][edge->head].push_back(edge);
                answer[edge->forest][edge->tail].push_back(edge);
            }
        }
    }

    return answer;
}

std::vector<std::shared_ptr<DirectedEdge>> Digraph::UnlabelledFundamentalCycle(const std::shared_ptr<DirectedEdge> &edge,
                                                                               int forest_index,
                                                                               std::vector<std::unordered_set<int>> &labelled_vertices,
                                                                               std::vector<int> &labelled_roots) {
    // returns unlabelled edges on the fundamental cycle of edge
    // ordered from L_i to some end of the edge
    // also updates labelled_vertices and labelled_roots
    // if there is a joining edge on the path, returns just that edge

    int vertex = 0;
    if (labelled_vertices[forest_index].find(edge->head) == labelled_vertices[forest_index].end()) {
        vertex = edge->head;
        if (labelled_vertices[forest_index].find(edge->tail) == labelled_vertices[forest_index].end()) {
            PrintGraph();
            throw std::runtime_error("in UnlabelledFundamentalCycle: both ends are unlabelled");
        }
    } else {
        if (labelled_vertices[forest_index].find(edge->tail) != labelled_vertices[forest_index].end()) {
            // both ends of the edge already in L_i
            return {};
        }
        vertex = edge->tail;
    }

    labelled_vertices[forest_index].insert(vertex);
    std::vector<std::shared_ptr<DirectedEdge>> path_above_vertex;
    std::vector<std::shared_ptr<DirectedEdge>> path_above_root;
    while (vertex != labelled_roots[forest_index]) {
        if (depth[forest_index][vertex] >= depth[forest_index][labelled_roots[forest_index]]) {
            // moving vertex up
            if (edge_to_parent[forest_index][vertex] == nullptr) {
                throw std::runtime_error("in UnlabelledFundamentalCycle: reached root, still not converged the cycle");
            }
            if (EdgeIsJoining(edge_to_parent[forest_index][vertex]) != -1) {
                return {edge_to_parent[forest_index][vertex]};
            }
            path_above_vertex.push_back(edge_to_parent[forest_index][vertex]);
            if (labelled_vertices[forest_index].find(edge_to_parent[forest_index][vertex]->AnotherVertex(vertex))
                != labelled_vertices[forest_index].end()) {
                // cycle collided with the labelled subtree and it is not root
                std::reverse(path_above_vertex.begin(), path_above_vertex.end());
                return path_above_vertex;
            }
            vertex = edge_to_parent[forest_index][vertex]->AnotherVertex(vertex);
            labelled_vertices[forest_index].insert(vertex);
        } else {
            // moving labelled root up
            if (edge_to_parent[forest_index][labelled_roots[forest_index]] == nullptr) {
                throw std::runtime_error("in UnlabelledFundamentalCycle: reached root, still not converged the cycle");
            }
            if (EdgeIsJoining(edge_to_parent[forest_index][labelled_roots[forest_index]]) != -1) {
                return {edge_to_parent[forest_index][labelled_roots[forest_index]]};
            }
            path_above_root.push_back(edge_to_parent[forest_index][labelled_roots[forest_index]]);
            labelled_roots[forest_index] =
                edge_to_parent[forest_index][labelled_roots[forest_index]]->AnotherVertex(labelled_roots[forest_index]);
            labelled_vertices[forest_index].insert(labelled_roots[forest_index]);
        }
    }

    std::reverse(path_above_vertex.begin(), path_above_vertex.end());
    path_above_root.insert(path_above_root.end(), path_above_vertex.begin(), path_above_vertex.end());
    return path_above_root;
}


