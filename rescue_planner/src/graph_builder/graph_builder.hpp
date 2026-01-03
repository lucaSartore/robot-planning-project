#pragma once
#include "../interface/interface.hpp"
#include <vector>
#include <iostream>


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
    void add_adjacent(int a, int b);
    Graph(vector<Point> points);
    friend std::ostream &operator<<(std::ostream &os, Graph &g);
};

class GraphBuilder {
public:
    virtual Graph convert(Map map) {};
};