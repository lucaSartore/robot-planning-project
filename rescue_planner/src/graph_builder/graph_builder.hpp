#pragma once
#include "../interface/interface.hpp"
#include <vector>
#include <iostream>
#include <exception>


class Node {
public:
    Point value;
    vector<int> adjacent;
    explicit Node(Point value);
    void add_adjacent(int value);
    friend std::ostream &operator<<(std::ostream &os, Node &n);
};
class Graph {
public:
    vector<Node> nodes;
    int exit_node;
    int robot_position;
    vector<int> victims_odes;
    void add_adjacent(int a, int b);
    Graph(vector<Point> points, int exit_node, int robot_position, vector<int> victims_odes);
    friend std::ostream &operator<<(std::ostream &os, Graph &g);
};

class GraphBuilder {
public:
    virtual Graph convert(Map map) {
        throw std::logic_error("GraphBuilder::convert(): not implemented");
    };
};