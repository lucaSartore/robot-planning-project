#pragma once
#include <unordered_map>
#include <vector>
#include "../interface/interface.hpp"
#include "../trajectory_planner/trajectory_planner.hpp"
#include "../graph_builder/graph_builder.hpp"

using namespace  std;

class DubinsNode;

class DubinsEdge {
public:
    DubinsNode* node;
    float arriving_angle;
    ExecutableDubinsTrajectory trajectory;
    DubinsEdge(DubinsNode* node, float arriving_angle, ExecutableDubinsTrajectory trajectory);
};

class DubinsNode {
public:
    int id;
    unordered_map<float, vector<DubinsEdge>> connections;
    explicit DubinsNode(int id);
    DubinsNode();
    void add_connection(float angle, DubinsEdge edge);
};

class DubinsGraph {
public:
    DubinsGraph(
        Map& map,
        OccupationApproximation& occupation_approximation,
        Graph& graph,
        float velocity,
        float k
    );
    float velocity;
    float k;
    unordered_map<int, DubinsNode> nodes;
    OccupationApproximation& occupation_approximation;
    Map& map;
    Graph& graph;
private:
    void insert_nodes_in_graph();
    void generate_edges(vector<float> const& angles);
    void generate_edge(int node, vector<float> const& angles);
    void generate_edge(int node, vector<float> const& starting_angles, vector<float> const& ending_angles);
    void generate_edge(int node, float starting_angle, float ending_angle);
    void generate_edge(int node_from, int node_to, float starting_angle, float ending_angle);

};