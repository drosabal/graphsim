#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <chrono>
#include <cstdint>
#include "graph.hpp"
#include "graph_builder.hpp"

using namespace std;

bool valid_args(int argc, char *argv[]);
void print_verts(const vector<int> &verts, ostream &output);
void print_error();

int main(int argc, char *argv[]) {
    if (!valid_args(argc, argv)) {
        print_error();
        return 1;
    }

    string graph_algorithm = argv[1];
    string graph_type = argv[2];
    Graph *graph = new Graph();
    int64_t build_time;
    int64_t run_time;
    ofstream output_file;

    auto t1 = chrono::high_resolution_clock::now();
    if (graph_type == "cycle") {
        int n = stoi(argv[3]);
        graph_builder::cycle(graph, n);
    } else if (graph_type == "complete") {
        int n = stoi(argv[3]);
        graph_builder::complete(graph, n);
    } else if (graph_type == "empty") {
        int n = stoi(argv[3]);
        graph_builder::empty(graph, n);
    } else if (graph_type == "heap") {
        int n = stoi(argv[3]);
        graph_builder::heap(graph, n);
    } else if (graph_type == "trunc-heap") {
        int m = stoi(argv[3]);
        int n = stoi(argv[4]);
        graph_builder::trunc_heap(graph, m, n);
    } else if (graph_type == "equiv-mod") {
        int k = stoi(argv[3]);
        int n = stoi(argv[4]);
        graph_builder::equiv_mod(graph, k, n);
    }
    auto t2 = chrono::high_resolution_clock::now();
    build_time = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

    output_file.open("output.txt");
    ostream &output = cout;
    if (graph_algorithm == "components") {
        vector<vector<int>> components = graph->connected_components();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        for (const vector<int> &component : components) {
            print_verts(component, output);
        }
    } else if (graph_algorithm == "cycle") {
        vector<int> cycle = graph->one_cycle();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        print_verts(cycle, output);
    } else if (graph_algorithm == "paths") {
        map<int, map<int, vector<int>>> all_paths = graph->shortest_paths();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        for (const auto &p1 : all_paths) {
            const map<int, vector<int>> &paths = p1.second;
            for (const auto &p2 : paths) {
                const vector<int> &path = p2.second;
                print_verts(path, output);
            }
        }
    }
    output_file.close();
    delete graph;

    cout << "Graph build time: " << build_time << "ms" << endl;
    cout << "Algorithm run time: " << run_time << "ms" << endl;

    return 0;
}

bool valid_args(int argc, char *argv[]) {
    if (argc != 4 && argc != 5) {
        return false;
    }

    string graph_algorithm = argv[1];
    string graph_type = argv[2];

    if (
        graph_algorithm != "components"
        && graph_algorithm != "cycle"
        && graph_algorithm != "paths"
    ) {
        return false;
    }
    
    if (
        graph_type == "cycle"
        || graph_type == "complete"
        || graph_type == "empty"
        || graph_type == "heap"
    ) {
        if (argc != 4) {
            return false;
        }
        int n;
        try {
            n = stoi(argv[3]);
        } catch (const invalid_argument &e) {
            return false;
        }
    } else if (
        graph_type == "trunc-heap"
        || graph_type == "equiv-mod"
    ) {
        if (argc != 5) {
            return false;
        }
        int mk, n;
        try {
            mk = stoi(argv[3]);
            n = stoi(argv[4]);
        } catch (const invalid_argument &e) {
            return false;
        }
    } else {
        return false;
    }

    return true;
}

void print_verts(const vector<int> &verts, ostream &output) {
    ostringstream line;
    line << "[";
    for (int i = 0; i < verts.size(); i++) {
        line << verts[i];
        if (i < verts.size() - 1) {
            line << ", ";
        }
    }
    line << "]\n";
    output << line.str();
}

void print_error() {
    cout << "Usage:" << endl;
    cout << "graphsim {components|cycle|paths} {cycle|complete|empty|heap} <n>" << endl;
    cout << "graphsim {components|cycle|paths} trunc-heap <m> <n>" << endl;
    cout << "graphsim {components|cycle|paths} equiv-mod <k> <n>" << endl;
}
