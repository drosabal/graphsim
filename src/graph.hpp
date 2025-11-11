#pragma once

#include <vector>
#include <map>
#include <stack>
#include <queue>
#include <unordered_set>
#include <utility>
#include <functional>
#include <algorithm>
#include <limits>
#include <cmath>

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
    map<int, vector<int>> shortest_paths(int s);
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

map<int, vector<int>> Graph::shortest_paths(int s) {
    map<int, vector<int>> paths;
    vector<int> verts;
    for (auto &p : adj) {
        verts.push_back(p.first);
    }

    const double INF = numeric_limits<double>::infinity();

    map<int, double> dist;
    map<int, int> prev;
    for (int v : verts) {
        dist[v] = INF;
    }
    dist[s] = 0.0;

    priority_queue<
        pair<double, int>,
        vector<pair<double, int>>,
        greater<pair<double, int>>
    > pq;
    pq.push({0.0, s});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) {
            continue;
        }
        for (auto [nei, w] : adj[u]) {
            double nd = d + w;
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
            //reverse(path.begin(), path.end());
            paths[t] = path;
        }
    }

    return paths;
}

map<int, map<int, vector<int>>> Graph::all_shortest_paths() {
    map<int, map<int, vector<int>>> all_paths;
    vector<int> verts;
    for (auto &p : adj) {
        verts.push_back(p.first);
    }

    const double INF = numeric_limits<double>::infinity();

    for (int s : verts) {
        map<int, double> dist;
        map<int, int> prev;
        for (int v : verts) {
            dist[v] = INF;
        }
        dist[s] = 0.0;

        priority_queue<
            pair<double, int>,
            vector<pair<double, int>>,
            greater<pair<double, int>>
        > pq;
        pq.push({0.0, s});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();
            if (d > dist[u]) {
                continue;
            }
            for (auto [nei, w] : adj[u]) {
                double nd = d + w;
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
                //reverse(path.begin(), path.end());
                all_paths[s][t] = path;
            }
        }
    }

    return all_paths;
}

pair<int, int> Graph::closest_pair() {
    int n = coords.size();
    if (n < 2) {
        return {-1, -1};
    }

    // Create a vector of indices and sort by x-coordinate
    vector<int> idx(n);
    for (int i = 0; i < n; ++i) {
        idx[i] = i;
    }

    sort(idx.begin(), idx.end(), [&](int a, int b) {
        return (
            coords[a].first < coords[b].first
            || (coords[a].first == coords[b].first && coords[a].second < coords[b].second)
        );
    });

    // Helper function to compute squared distance
    auto dist_sq = [&](int i, int j) -> double {
        double dx = coords[i].first - coords[j].first;
        double dy = coords[i].second - coords[j].second;
        return dx * dx + dy * dy;
    };

    // Recursive function
    function<pair<int, int>(int, int)> closest_util = [&](int left, int right) -> pair<int, int> {
        if (right - left <= 3) {
            // Brute force for small subsets
            pair<int, int> best = {-1, -1};
            double min_dist = numeric_limits<double>::max();
            for (int i = left; i < right; i++) {
                for (int j = i + 1; j < right; j++) {
                    double d = dist_sq(idx[i], idx[j]);
                    if (d < min_dist) {
                        min_dist = d;
                        best = {idx[i], idx[j]};
                    }
                }
            }
            return best;
        }

        int mid = (left + right) / 2;
        int mid_idx = idx[mid];

        auto dl = closest_util(left, mid);
        auto dr = closest_util(mid, right);

        double d1 = (dl.first == -1) ? numeric_limits<double>::max() : dist_sq(dl.first, dl.second);
        double d2 = (dr.first == -1) ? numeric_limits<double>::max() : dist_sq(dr.first, dr.second);
        double delta = min(d1, d2);
        pair<int, int> best = (d1 < d2) ? dl : dr;

        // Build strip around mid point
        vector<int> strip;
        for (int i = left; i < right; i++) {
            double dx = coords[idx[i]].first - coords[mid_idx].first;
            if (dx * dx < delta) {
                strip.push_back(idx[i]);
            }
        }

        // Sort strip by y-coordinate
        sort(strip.begin(), strip.end(), [&](int a, int b) {
            return coords[a].second < coords[b].second;
        });

        // Check strip for closer pairs
        for (int i = 0; i < strip.size(); i++) {
            for (
                int j = i + 1;
                (
                    j < strip.size()
                    && (
                        (coords[strip[j]].second - coords[strip[i]].second)
                        * (coords[strip[j]].second - coords[strip[i]].second)
                        < delta
                    )
                );
                j++
            ) {
                double d = dist_sq(strip[i], strip[j]);
                if (d < delta) {
                    delta = d;
                    best = {strip[i], strip[j]};
                }
            }
        }

        return best;
    };

    auto result = closest_util(0, n);

    // Return indices in sorted order (smaller index first)
    if (result.first > result.second) {
        swap(result.first, result.second);
    }
    return result;
}
