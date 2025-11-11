#pragma once

#include <random>
#include <cmath>
#include "graph.hpp"

namespace graph_builder {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> random_weight(0.0, 1.0);

    void empty(Graph *graph, int n) {
        for (int i = 0; i < n; i++) {
            graph->add_vertex(i);
        }
    }

    void cycle(Graph *graph, int directed, int weighted, int n) {
        for (int i = 0; i < n - 1; i++) {
            double w = (weighted) ? random_weight(gen) : 1.0;
            graph->add_edge(i, i + 1, w);
            if (!directed) {
                graph->add_edge(i + 1, i, w);  
            }
        }
        double w = (weighted) ? random_weight(gen) : 1.0;
        graph->add_edge(n - 1, 0, w);
        if (!directed) {
            graph->add_edge(0, n - 1, w);  
        }
    }

    void grid_2d(Graph *graph, int directed, int weighted, int n) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                int u = i * n + j;

                // Right neighbor: (i, j + 1)
                if (j + 1 < n) {
                    int v = i * n + (j + 1);
                    double w = (weighted) ? random_weight(gen) : 1.0;
                    graph->add_edge(u, v, w);
                    if (directed) {
                        w = (weighted) ? random_weight(gen) : 1.0;
                    }
                    graph->add_edge(v, u, w);
                }

                // Down neighbor: (i + 1, j)
                if (i + 1 < n) {
                    int v = (i + 1) * n + j;
                    double w = (weighted) ? random_weight(gen) : 1.0;
                    graph->add_edge(u, v, w);
                    if (directed) {
                        w = (weighted) ? random_weight(gen) : 1.0;
                    }
                    graph->add_edge(v, u, w);
                }
            }
        }
    }

    void complete(Graph *graph, int directed, int weighted, int n) {
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double w = (weighted) ? random_weight(gen) : 1.0;
                graph->add_edge(i, j, w);
                if (directed) {
                    w = (weighted) ? random_weight(gen) : 1.0;
                }
                graph->add_edge(j, i, w);
            }
        }
    }

    void euclidean_complete(Graph *graph, int n) {
        for (int i = 0; i < n; i++) {
            double x = random_weight(gen);
            double y = random_weight(gen);
            graph->add_vertex(i);
            graph->add_coords(x, y);
        }
        /*
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                auto [x1, y1] = graph->get_coords(i);
                auto [x2, y2] = graph->get_coords(j);
                double w = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
                graph->add_edge(i, j, w);
                graph->add_edge(j, i, w);
            }
        }
        */
    }

}
