#include "graph_builder.hpp"
#include "../util/display.hpp"
#include <unordered_set>

Node::Node(Point value) : value(value) {}

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
    for (const auto& p : points) {
        this->nodes.emplace_back(p);
    }
    this->exit_node = exit_node;
    this -> robot_position = robot_position;
    this->victims_odes = victims_odes;
}

tuple<vector<tuple<Point, Point>>, vector<Point>> Graph::get_debug_data() {
    vector<tuple<Point,Point>> lines;
    vector<Point> points;
    for (const auto& p : nodes) {
        points.push_back(p.value);
        for (const auto& v : p.adjacent) {
            lines.push_back({p.value, nodes[v].value});
        }
    }
    return {lines, points};
}
void Graph::debug() {
    auto x = this->get_debug_data();
    display(std::get<0>(x), std::get<1>(x));
}


void Graph::add_adjacent(int a, int b) {
    if (a >= 0 && static_cast<size_t>(a) < nodes.size()) {
        nodes[a].add_adjacent(b);
    }
}

std::ostream &operator<<(std::ostream &os, Graph &g) {
    os << "Graph Contents:\n";
    for (size_t i = 0; i < g.nodes.size(); ++i) {
        os << "  Index " << i << ": " << g.nodes[i] << "\n";
    }
    return os;
}