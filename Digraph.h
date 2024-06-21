#ifndef K_FORESTS_BOUNDED_INDEGREE_DIGRAPH_H
#define K_FORESTS_BOUNDED_INDEGREE_DIGRAPH_H

#include <vector>
#include <memory>
#include <unordered_set>
#include <unordered_map>

#include "DirectedEdge.h"
#include "DisjointSets.h"

class Digraph {
 public:
    int num_forests;

    Digraph(const std::vector<std::vector<int>> &adj_list);

    void PrintGraph();

    void GenerateForests(int k);

    std::vector<std::vector<std::pair<int, int>>> GetForests();

    void Validate();

 private:
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> adj_list_;
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> adj_list_inv_;
    // adj_list_inv_[i] is a vector of edges with head = i
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> adj_list_full_;
    // adj_list_full_ is an undirected adjacency list
    int num_edges;

    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> edge_to_parent;
    // edge_to_parent[i][j] is an edge to the parent of vertex j in i-th tree, nullptr if j is root
    std::vector<std::vector<int>> depth;
    // depth[i][j] is the depth of vertex j in the i-th tree

    std::vector<bool> vertex_active;

    std::vector<DisjointSets> disjoint_components;
    std::mt19937 generator;

    int EdgeIsJoining(const std::shared_ptr<DirectedEdge> &edge);

    void GenerateNextForest();

    void AugmentPath(const std::vector<std::shared_ptr<DirectedEdge>> &path);

    std::vector<std::shared_ptr<DirectedEdge>>
    FundamentalCycleLegacy(const std::shared_ptr<DirectedEdge> &edge, int forest_index);

    std::vector<std::shared_ptr<DirectedEdge>> NeighborsLegacy(const std::shared_ptr<DirectedEdge> &edge);

    std::vector<std::shared_ptr<DirectedEdge>> AugmentingPathLegacy(int vertex);

    std::vector<std::shared_ptr<DirectedEdge>> Search(int vertex, int bound);

    std::vector<std::shared_ptr<DirectedEdge>> UnlabelledFundamentalCycle(const std::shared_ptr<DirectedEdge>& edge,
                                                                          int forest_index,
                                                                          std::vector<std::unordered_set<int>>& labelled_vertices,
                                                                          std::vector<int>& labelled_roots);

    bool Epoch(int bound);

    std::vector<int> ActiveVerticesPerComponent();

    void UpdateDepthsAndParents();

    void UpdateActivity();

    int Indeg(int vertex);

    int Deficit(int vertex);

    int TotalModifiedDeficit();

    std::vector<std::vector<std::vector<std::shared_ptr<DirectedEdge>>>> FullAdjListPerForest();
};

#endif
