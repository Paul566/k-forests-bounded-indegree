cmake_minimum_required(VERSION 3.28)
project(k_forests_bounded_indegree)

set(CMAKE_CXX_STANDARD 17)

add_executable(k_forests_bounded_indegree main.cpp
        Digraph.h
        Digraph.cpp
        DirectedEdge.h
        DirectedEdge.cpp
        DisjointSets.h
        DisjointSets.cpp)
