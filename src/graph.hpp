#pragma once

#include <vector>
#include <map>
#include <stack>
#include <queue>
#include <unordered_set>
#include <utility>
#include <algorithm>
#include <climits>

using namespace std;

class Graph {

private:
    map<int, vector<pair<int, double>>> adj;
    vector<pair<double, double>> coords;

    void dfs_component(int start, unordered_set<int> &visited, vector<int> &component);
    void dfs_cycle(int start, unordered_set<int> &visited, vector<int> &cycle);

public:
    void add_vertex(int v);
    void add_edge(int u, int v, double w);
    void add_coords(double x, double y);
    pair<double, double> get_coords(int v);
    vector<vector<int>> connected_components();
    vector<int> one_cycle();
    map<int, vector<int>> shortest_paths(int source);
    map<int, map<int, vector<int>>> all_shortest_paths();
    pair<int, int> closest_pair();

};

void Graph::add_vertex(int v) {
    adj.emplace(v, vector<pair<int, double>>());
}

void Graph::add_edge(int u, int v, double w) {
    adj[u].push_back({v, w});
}

void Graph::add_coords(double x, double y) {
    coords.push_back({x, y});
}

pair<double, double> Graph::get_coords(int v) {
    return coords[v];
}

vector<vector<int>> Graph::connected_components() {
    vector<vector<int>> components;
    unordered_set<int> visited;
    for (const auto &p : adj) {
        int v = p.first;
        if (visited.find(v) == visited.end()) {
            vector<int> component;
            dfs_component(v, visited, component);
            components.push_back(component);
        }
    }
    return components;
}

void Graph::dfs_component(int start, unordered_set<int> &visited, vector<int> &component) {
    stack<int> stk;
    stk.push(start);
    visited.insert(start);
    while (!stk.empty()) {
        int v = stk.top();
        stk.pop();
        component.push_back(v);
        for (auto [nei, w] : adj[v]) {
            if (visited.find(nei) == visited.end()) {
                stk.push(nei);
                visited.insert(nei);
            }
        }
    }
}

vector<int> Graph::one_cycle() {
    unordered_set<int> visited;
    for (const auto &p : adj) {
        int v = p.first;
        if (visited.find(v) == visited.end()) {
            vector<int> cycle;
            dfs_cycle(v, visited, cycle);
            if (!cycle.empty()) {
                return cycle;
            }
        }
    }
    return vector<int>();
}

void Graph::dfs_cycle(int start, unordered_set<int> &visited, vector<int> &cycle) {
    struct Frame {
        int vertex;
        int parent;
        size_t nextNei;
        Frame(int v, int p) : vertex(v), parent(p), nextNei(0) {}
    };

    stack<Frame> stk;
    vector<int> path;
    stk.emplace(start, -1);
    visited.insert(start);
    path.push_back(start);

    while (!stk.empty()) {
        Frame &f = stk.top();
        const vector<pair<int, double>> &neighbors = adj[f.vertex];
        bool pushed = false;

        for (; f.nextNei < neighbors.size(); f.nextNei++) {
            int nei = neighbors[f.nextNei].first;

            if (nei == f.parent) {
                continue;
            }

            if (visited.find(nei) != visited.end()) {
                auto it = find(path.begin(), path.end(), nei);
                if (it != path.end()) {
                    cycle.assign(it, path.end());
                    cycle.push_back(nei);
                    return;
                }
                continue;
            }

            stk.emplace(nei, f.vertex);
            visited.insert(nei);
            path.push_back(nei);
            f.nextNei++;
            pushed = true;
            break;
        }

        if (pushed) {
            continue;
        }

        path.pop_back();
        stk.pop();
    }
}

map<int, vector<int>> Graph::shortest_paths(int source) {
    return {};
}

map<int, map<int, vector<int>>> Graph::all_shortest_paths() {
    map<int, map<int, vector<int>>> all_paths;
    vector<int> verts;
    for (auto& p : adj) {
        verts.push_back(p.first);
    }

    const long long INF = LLONG_MAX / 2;

    for (int s : verts) {
        map<int, long long> dist;
        map<int, int> prev;
        for (int v : verts) {
            dist[v] = INF;
        }
        dist[s] = 0;

        priority_queue<
            pair<long long, int>,
            vector<pair<long long, int>>,
            greater<pair<long long, int>>
        > pq;
        pq.push({0, s});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();
            if (d > dist[u]) {
                continue;
            }
            for (auto [nei, w] : adj[u]) {
                long long nd = d + 1;
                if (nd < dist[nei]) {
                    dist[nei] = nd;
                    prev[nei] = u;
                    pq.push({nd, nei});
                }
            }
        }
        
        for (int t : verts) {
            if (dist[t] < INF) {
                vector<int> path;
                int cur = t;
                while (true) {
                    path.push_back(cur);
                    if (cur == s) {
                        break;
                    }
                    if (prev.find(cur) == prev.end()) {
                        break;
                    }
                    cur = prev[cur];
                }
                reverse(path.begin(), path.end());
                all_paths[s][t] = path;
            }
        }
    }

    return all_paths;
}

pair<int, int> Graph::closest_pair() {
    return {};
}
