#pragma once
#include <math.h>
#include <unordered_map>
#include <vector>
#include "../interface/common_types.hpp"
#include "../trajectory_planner/trajectory_planner.hpp"
#include "../graph_builder/graph_builder.hpp"
#include "../util/hashable_tuple.hpp"
#include <mutex>
#include <optional>

using namespace  std;

class Result {
public:
    vector<ExecutableDubinsTrajectory> trajectory;
    /// order of node visited + angle
    vector<tuple<int, float>> nodes_order;
    /// first element is node index, second is position and value
    vector<tuple<int,Victim>> victims;
    float total_length;
    float total_time;
    float total_value;
    Result(vector<ExecutableDubinsTrajectory> trajectory, vector<tuple<int, Victim>> victims, vector<tuple<int, float>> nodes_order);
    vector<Pose> get_full_trajectory(int resolution = 10);
    tuple<Pose, Velocities> get_at(float time);
};

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
    mutex connections_mutex;
    unordered_map<float, vector<DubinsEdge>> connections;
    explicit DubinsNode(int id);
    DubinsNode();
    DubinsNode(const DubinsNode &other);
    DubinsNode &operator=(const DubinsNode &other);
    void add_connection(float angle, DubinsEdge edge);
};

class DubinsGraph {
public:
    DubinsGraph(
        Map& map,
        OccupationApproximation& occupation_approximation,
        Graph& graph,
        float velocity,
        float k,
        optional<Result> initial_guess = {},
        float initial_guess_search_range = M_PI/4
    );
    float velocity;
    float k;
    optional<Result> initial_guess;
    float initial_guess_search_range;
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
    void insert_nodes_in_graph_with_initial_guess();
    void generate_edges_with_initial_guess();
};


class GraphSearch {
public:
    GraphSearch(DubinsGraph& graph, vector<int> victims);
    tuple<vector<tuple<int, float>>, vector<ExecutableDubinsTrajectory>> execute();
    void debug(vector<ExecutableDubinsTrajectory> path);
private:
    /// graph
    DubinsGraph & graph;
    /// initial node
    DubinsNode * start;
    /// list of nodes to visit (victims and finally exit)
    vector<DubinsNode *> to_visit;
    /// costs associated with reaching the end
    /// starting from one of the nodes in "to_visit"
    unordered_map<int, float> heuristic_costs;
    float heuristic_cost(int node_start, int next_objective);
    tuple<vector<tuple<int, float>>, vector<ExecutableDubinsTrajectory>> build_solution(unordered_map<tuple<int, float,int>, tuple<int, float, int>, hash_triplet> & backtracking, tuple<int, float, int> end);
};



class RescueOrderSearch {
public:
    explicit RescueOrderSearch(DubinsGraph& graph);
    void  execute();
    Result get_best_solution(float time_limit);
    void refine_all_solutions(float range);
    void debug(Result r);
private:
    /// graph
    DubinsGraph & graph;
    vector<Result> results;
};

