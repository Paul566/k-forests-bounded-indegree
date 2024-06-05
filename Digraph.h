#ifndef K_FORESTS_BOUNDED_INDEGREE_DIGRAPH_H
#define K_FORESTS_BOUNDED_INDEGREE_DIGRAPH_H


#include <vector>
#include <memory>

#include "DirectedEdge.h"
#include "DisjointSets.h"

class Digraph {
public:
    int num_forests;

    Digraph(const std::vector<std::vector<int>> &adj_list);

    void PrintGraph();

    void GenerateForests(int k);

    std::vector<std::vector<std::pair<int, int>>> GetForests();

    std::vector<int> ListOfComponentDeficits();

private:
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> adj_list_;
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> adj_list_inv_;
    // adj_list_inv_[i] is a vector of edges with head = i
    std::vector<std::vector<std::shared_ptr<DirectedEdge>>> adj_list_full_;
    // adj_list_full_ is an undirected adjacency list

    std::vector<DisjointSets> disjoint_components;
    std::mt19937 generator;

    int EdgeIsJoining(const std::shared_ptr<DirectedEdge> &edge);

    void GenerateNextForest();

    void AugmentPath(const std::vector<std::shared_ptr<DirectedEdge>> &path);

    std::vector<std::shared_ptr<DirectedEdge>>
    FundamentalCycle(const std::shared_ptr<DirectedEdge> &edge, int forest_index);

    std::vector<std::shared_ptr<DirectedEdge>> Neighbors(const std::shared_ptr<DirectedEdge>& edge);

    std::vector<std::shared_ptr<DirectedEdge>> AugmentingPath(int vertex);

    int Indeg(int vertex);

    int Deficit(int vertex);
};


#endif
