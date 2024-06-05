#include <stdexcept>
#include "DisjointSets.h"


DisjointSets::DisjointSets(int size) {
    ranks = std::vector<int>(size, 0);
    parents.reserve(size);
    for (int i = 0; i < size; ++i) {
        parents.push_back(i);
    }
}

int DisjointSets::Representative(int element) {
    if ((element < 0) || (element >= static_cast<int>(ranks.size()))) {
        throw std::runtime_error("element not in disjoint sets");
    }

    int grandparent = element;
    while (parents[grandparent] != grandparent) {
        grandparent = parents[grandparent];
    }

    int current_element = element;
    while (parents[current_element] != grandparent) {
        int next_element = parents[current_element];
        parents[current_element] = grandparent;
        current_element = next_element;
    }

    return grandparent;
}

void DisjointSets::UniteRepresentatives(int first, int second) {
    if (first == second) {
        return;
    }

    if (ranks[first] < ranks[second]) {
        parents[first] = second;
        return;
    }

    if (ranks[first] > ranks[second]) {
        parents[second] = first;
        return;
    }

    parents[first] = second;
    ++ranks[second];
}

void DisjointSets::Unite(int first, int second) {
    UniteRepresentatives(Representative(first), Representative(second));
}
