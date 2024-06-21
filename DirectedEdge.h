#ifndef K_FORESTS_BOUNDED_INDEGREE_DIRECTEDEDGE_H
#define K_FORESTS_BOUNDED_INDEGREE_DIRECTEDEDGE_H


class DirectedEdge {
public:
    int forest;
    // the index of a forest this edge is in, -1 if not covered
    // forest are enumerated from 0
    // TODO use std::optional<int> instead of the magical constant "-1"

    DirectedEdge(int head_, int tail_) : head(head_), tail(tail_) {
        forest = -1;
    }

    int AnotherVertex(int vertex) const {
        if (vertex == head) {
            return tail;
        }
        return head;
    }

    const int head, tail;
};


#endif
