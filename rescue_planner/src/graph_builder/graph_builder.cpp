#include "graph_builder.hpp"
#include "../util/display.hpp"
#include <unordered_set>
#include <algorithm>
#include <set>

Node::Node(Point value) : value(value) {}
Node::Node() : value(Point(0,0)) {}

void Node::add_adjacent(int value) {
    this->adjacent.push_back(value);
}

std::ostream &operator<<(std::ostream &os, Node &n) {
    os << "Node(Value: " << n.value << ", Neighbors: [";
    for (size_t i = 0; i < n.adjacent.size(); ++i) {
        os << n.adjacent[i] << (i == n.adjacent.size() - 1 ? "" : ", ");
    }
    os << "])";
    return os;
}

Graph::Graph(vector<Point> points, int exit_node, int robot_position, vector<int> victims_odes) {
    this->nodes = {};
    int i = 0;
    for (const auto& p : points) {
        this->nodes[i] = Node(p);
        i += 1;
    }
    this->exit_node = exit_node;
    this ->robot_position = robot_position;
    this->victims_odes = victims_odes;
}

tuple<vector<tuple<Point, Point>>, vector<Point>> Graph::get_debug_data() {
    vector<tuple<Point,Point>> lines;
    vector<Point> points;
    for (const auto& p : nodes) {
        points.push_back(p.second.value);
        for (const auto& v : p.second.adjacent) {
            lines.push_back({p.second.value, nodes[v].value});
        }
    }
    return {lines, points};
}
void Graph::debug() {
    auto x = this->get_debug_data();
    display(std::get<0>(x), std::get<1>(x));
}


void Graph::add_adjacent(int a, int b) {
    nodes[a].add_adjacent(b);
    nodes[b].add_adjacent(a);
}

std::ostream &operator<<(std::ostream &os, Graph &g) {
    os << "Graph Contents:\n";
    for (size_t i = 0; i < g.nodes.size(); ++i) {
        os << "  Index " << i << ": " << g.nodes[i] << "\n";
    }
    return os;
}


void Graph::add_skip_ahead_connections() {
    unordered_map<int, vector<int>> nodes_to_add;
    for (auto i: this->nodes) {
        int id = i.first;
        auto node = i.second;
        vector<int> to_add;
        for (int adj: node.adjacent ) {
            Node* adj_node = &nodes[adj];
            for (int adj2: adj_node->adjacent) {
                to_add.push_back(adj2);
            }
        }
        nodes_to_add[id] = to_add;
    }
    for (auto &i: this->nodes) {
        int id = i.first;
        Node node = i.second;
        auto to_add = nodes_to_add[id];
        for (int adj: node.adjacent) {
            to_add.push_back(adj);
        }
        auto to_add_unique = unordered_set(to_add.begin(), to_add.end());
        to_add_unique.erase(id);
        i.second.adjacent = vector(to_add_unique.begin(), to_add_unique.end());
    }
}
