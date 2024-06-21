#include <iostream>
#include <vector>
#include <random>

#include "Digraph.h"


std::vector<std::vector<int>> RandomGraph(int num_vertices, int num_edges, std::mt19937 &generator) {
    // returns adjacency list of a random graph
    // multiple edges allowed, loops not allowed
    std::vector<std::vector<int>> adj_list(num_vertices);
    std::uniform_int_distribution<int> distribution(0, num_vertices - 1);
    int edges_generated = 0;
    while (edges_generated < num_edges) {
        int head = distribution(generator);
        int tail = distribution(generator);
        if (head != tail) {
            adj_list[tail].push_back(head);
            ++edges_generated;
        }
    }
    return adj_list;
}

void RunTest(int max_vertices, std::mt19937 &generator) {
    std::uniform_int_distribution<int> distribution_n(3, max_vertices);
    int num_vertices = distribution_n(generator);
    std::uniform_int_distribution<int> distribution_m(num_vertices - 1, num_vertices * (num_vertices - 1) / 2);
    int num_edges = distribution_m(generator);

    auto adj_list = RandomGraph(num_vertices, num_edges, generator);

    for (int k = 1; k <= num_vertices; ++k) {
        Digraph digraph(adj_list);
//        std::cout << "k: " << k << std::endl;
//        digraph.PrintGraph();

        digraph.GenerateForests(k);
        digraph.Validate();

        // digraph.PrintGraph();
    }
}

int main() {
    std::mt19937 generator(566);

    for (int i = 0; i < 1000; ++i) {
        if (i % 1 == 0) {
            std::cout << "i: " << i << std::endl;
        }
        RunTest(100, generator);
    }

//    std::vector<std::vector<int>> adj_list(5);
//    adj_list[0].push_back(4);
//    adj_list[0].push_back(1);
//    adj_list[1].push_back(2);
//    adj_list[2].push_back(0);
//    adj_list[2].push_back(1);
//    adj_list[2].push_back(1);
//    adj_list[2].push_back(3);
//    adj_list[4].push_back(0);
//    adj_list[4].push_back(2);
//
//    Digraph digraph(adj_list);
//    digraph.GenerateForests(2);
//    digraph.PrintGraph();
//    digraph.Validate();

    return 0;
}
