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

    for (int k = num_vertices; k <= num_vertices; ++k) {
//        std::cout << "\nk = " << k << std::endl;
//        std::cout << "Initial configuration:\n";
        Digraph digraph(adj_list);
//        digraph.PrintGraph();

        digraph.GenerateForests(k);
        auto deficits = digraph.ListOfComponentDeficits();

//        std::cout << "Final configuration:\n";
//        digraph.PrintGraph();
//        for (int d : deficits) {
//            std::cout << d << " ";
//        }

        for (int deficit : deficits) {
            if (deficit > k) {
                std::cout << "\n-----------------------------------------\n";
                std::cout << "example with large deficit found\n";
                digraph.PrintGraph();
                break;
            }
        }
    }
}

int main() {
    std::mt19937 generator(566);

    for (int i = 0; i < 100000; ++i) {
        if (i % 10 == 0) {
            std::cout << "i: " << i << std::endl;
        }
        RunTest(30, generator);
    }

//    std::vector<std::vector<int>> adj_list(4);
//    adj_list[0].push_back(2);
//    adj_list[0].push_back(3);
//    adj_list[2].push_back(3);
//    adj_list[2].push_back(0);
//    adj_list[3].push_back(0);
//    adj_list[3].push_back(0);
//
//    Digraph digraph(adj_list);
//    digraph.GenerateForests(3);

    return 0;
}
