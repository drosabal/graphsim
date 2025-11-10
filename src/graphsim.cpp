#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <utility>
#include <chrono>
#include <cstdint>
#include <cmath>
#include "graph.hpp"
#include "graph_builder.hpp"

using namespace std;

void print_verts(const vector<int> &verts, ostream &output);
int square_size(int n);

int main() {
    int directed = 0;
    int weighted = 0;
    int graph_type = 0;
    int graph_algo = 0;
    int n = 0;
    int source = 0;

    int64_t build_time;
    int64_t run_time;
    ofstream output_file;
    Graph *graph = new Graph();

    cout << "0: Undirected" << endl;
    cout << "1: Directed" << endl;
    cout << "Choose undirected or directed: ";
    cin >> directed;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << endl;

    cout << "0: Unweighted" << endl;
    cout << "1: Weighted (random weights Uniform(0, 1))" << endl;
    cout << "Choose unweighted or weighted: ";
    cin >> weighted;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << endl;

    cout << "0: Empty" << endl;
    cout << "1: Cycle" << endl;
    cout << "2: 2D Grid" << endl;
    cout << "3: Complete" << endl;
    if (!directed && weighted) {
        cout << "4: Euclidean Complete" << endl;
    }
    cout << "Choose graph type: ";
    cin >> graph_type;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Number of vertices: ";
    cin >> n;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << endl;
    if (graph_type == 2) {
        n = square_size(n);
    }

    cout << "0: Components" << endl;
    cout << "1: Cycle" << endl;
    cout << "2: Paths" << endl;
    cout << "3: All Paths" << endl;
    if (graph_type == 4 && !directed && weighted) {
        cout << "4: Closest Pair" << endl;
    }
    cout << "Choose graph algorithm: ";
    cin >> graph_algo;
    if (graph_algo == 2) {
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Source: ";
        cin >> source;
    }
    cout << endl;

    //
    // 0: Empty
    // 1: Cycle
    // 2: 2D Grid
    // 3: Complete
    // 4: Euclidean Complete
    //
    auto t1 = chrono::high_resolution_clock::now();
    if (graph_type == 0) {
        graph_builder::empty(graph, n);
    } else if (graph_type == 1) {
        graph_builder::cycle(graph, directed, weighted, n);
    } else if (graph_type == 2) {
        graph_builder::grid_2d(graph, directed, weighted, n);
    } else if (graph_type == 3) {
        graph_builder::complete(graph, directed, weighted, n);
    } else if (graph_type == 4) {
        graph_builder::euclidean_complete(graph, n);
    }
    auto t2 = chrono::high_resolution_clock::now();
    build_time = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

    //
    // 0: Components
    // 1: Cycle
    // 2: Paths
    // 3: All Paths
    // 4: Closest Pair
    //
    output_file.open("output.txt");
    ostream &output = cout;
    if (graph_algo == 0) {
        vector<vector<int>> components = graph->connected_components();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        for (const vector<int> &component : components) {
            print_verts(component, output);
        }
    } else if (graph_algo == 1) {
        vector<int> cycle = graph->one_cycle();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        print_verts(cycle, output);
    } else if (graph_algo == 2) {
        map<int, vector<int>> paths = graph->shortest_paths(source);
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        for (const auto &p : paths) {
            const vector<int> &path = p.second;
            print_verts(path, output);
        }
    } else if (graph_algo == 3) {
        map<int, map<int, vector<int>>> all_paths = graph->all_shortest_paths();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        for (const auto &p1 : all_paths) {
            const map<int, vector<int>> &paths = p1.second;
            for (const auto &p2 : paths) {
                const vector<int> &path = p2.second;
                print_verts(path, output);
            }
        }
    } else if (graph_algo == 4) {
        pair<int, int> p = graph->closest_pair();
        auto t3 = chrono::high_resolution_clock::now();
        run_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2).count();
        vector<int> verts;
        verts.push_back(p.first);
        verts.push_back(p.second);
        print_verts(verts, output);
    }
    output_file.close();
    delete graph;

    cout << "Graph build time: " << build_time << "ms" << endl;
    cout << "Algorithm run time: " << run_time << "ms" << endl;

    return 0;
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

int square_size(int n) {
    if (n < 0) {
        return 0;
    }
    int root = static_cast<int>(sqrt(n));
    if (root * root == n) {
        return root;
    }
    return root + 1;
}
