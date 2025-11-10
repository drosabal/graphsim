#pragma once

#include "graph.hpp"

namespace graph_builder {

    void cycle(Graph *graph, int n) {
        for (int i = 0; i < n - 1; i++) {
            graph->add_edge(i, i + 1);
        }
        graph->add_edge(n - 1, 0);
    }

    void complete(Graph *graph, int n) {
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                graph->add_edge(i, j);
            }
        }
    }

    void empty(Graph *graph, int n) {
        for (int i = 0; i < n; i++) {
            graph->add_vertex(i);
        }
    }

    void heap(Graph *graph, int n) {
    }

    void trunc_heap(Graph *graph, int m, int n) {
    }

    void equiv_mod(Graph *graph, int k, int n) {
    }

}
