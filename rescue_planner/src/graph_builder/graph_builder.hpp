#pragma once
#include "../interface/interface.hpp"
#include <vector>
#include <iostream>
#include <unordered_map>


class Node {
public:
    Point value;
    vector<int> adjacent;
    explicit Node(Point value);
    Node();
    void add_adjacent(int value);
    friend std::ostream &operator<<(std::ostream &os, Node &n);
};
class Graph {
public:
    unordered_map<int, Node> nodes;
    int exit_node;
    int robot_position;
    vector<int> victims_odes;
    void add_adjacent(int a, int b);
    Graph(vector<Point> points, int exit_node, int robot_position, vector<int> victims_odes);
    friend std::ostream &operator<<(std::ostream &os, Graph &g);
    void debug();
    tuple<vector<tuple<Point, Point>>, vector<Point>> get_debug_data();
    // add one extra node to the next node's next node
    void add_skip_ahead_connections();
};

class GraphBuilder {
public:
    virtual Graph convert(Map map) {
        throw std::logic_error("GraphBuilder::convert(): not implemented");
    };
};