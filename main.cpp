#include <iostream>
#include <vector>
#include <random>
#include <chrono>

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

//    auto adj_list = RandomGraph(100, 2000, generator);
//    Digraph digraph = Digraph(adj_list);
//
//    auto start = std::chrono::high_resolution_clock::now();
//    digraph.GenerateForests(20);
//    auto stop = std::chrono::high_resolution_clock::now();
//
//    double duration = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
//        stop - start).count()) / 1'000'000;
//    std::cout << duration;

    for (int i = 0; i < 1000; ++i) {
        if (i % 10 == 0) {
            std::cout << "i: " << i << std::endl;
        }
        RunTest(10, generator);
    }

    return 0;
}
