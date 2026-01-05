#pragma once
#include <unordered_map>
#include <vector>
#include "../interface/interface.hpp"
#include "../trajectory_planner/trajectory_planner.hpp"
#include "../graph_builder/graph_builder.hpp"

using namespace  std;

class DubinsEdge;

class DubinsNode {
public:
    int id;
    /// nodes (ordered by the starting angle)
    unordered_map<float, vector<DublinsEdge>> connections;
};

class DubinsEdge {
public:
    DubinsNode* node;
    float arriving_angle;
    ExecutableDubinsTrajectory trajectory;
};


class DubinsGraph {
public:
    unordered_map<int, DubinsNode> nodes;
    OccupationApproximation occupation_approximation;
    Map map;
    GraphBuilder graph_builder;
};