#include "dubins_graph.hpp"

#include <math.h>

#include "../trajectory_planner/trajectory_planner.hpp"


DubinsEdge::DubinsEdge(DubinsNode *node, float arriving_angle, ExecutableDubinsTrajectory trajectory) {
    this->arriving_angle = arriving_angle;
    this->trajectory = std::move(trajectory);
    this->node = node;
}

DubinsNode::DubinsNode(int id) {
    this->id = id;
    this->connections = {};
}

DubinsNode::DubinsNode() {
    id = -1;
    connections = {};
}


void DubinsNode::add_connection(float angle, DubinsEdge edge) {
    if (this->connections.find(angle) != this->connections.end()) {
        this->connections[angle] = {};
    }
    this->connections[angle].push_back(edge);
}


DubinsGraph::DubinsGraph(Map& map, OccupationApproximation& occupation_approximation, Graph& graph, float velocity, float k)
    :occupation_approximation(occupation_approximation), map(map), graph(graph) {
    this->velocity = velocity;
    this->k = k;
    this->nodes = {};
    this->insert_nodes_in_graph();
    this->generate_edges({
        0.00 * M_PI,
        0.25 * M_PI,
        0.50 * M_PI,
        0.75 * M_PI,
        1.00 * M_PI,
        1.00 * M_PI,
        1.25 * M_PI,
        1.50 * M_PI,
        1.75 * M_PI,
    });
}

void DubinsGraph::insert_nodes_in_graph() {
    for (auto i: this->graph.nodes) {
        auto id = i.first;
        auto node = i.second;
        this->nodes[id] = DubinsNode(id);
    }
}

void DubinsGraph::generate_edges(vector<float> const& angles) {
    for (auto const& i: graph.nodes) {
        auto id = i.first;
        generate_edge(id, angles);
    }
}

void DubinsGraph::generate_edge(int node, vector<float> const& angles) {
    return generate_edge(node, angles, angles);
}

void DubinsGraph::generate_edge(int node, vector<float> const& starting_angles, vector<float> const& ending_angles) {
    for (float start: starting_angles) {
        for (float end: ending_angles) {
            generate_edge(node, start, end);
        }
    }
}

void DubinsGraph::generate_edge(int node, float starting_angle, float ending_angle) {
    for (auto node_to: graph.nodes[node].adjacent) {
        generate_edge(node, node_to, starting_angle, ending_angle);
    }
}

void DubinsGraph::generate_edge(int node_from, int node_to, float starting_angle, float ending_angle) {
    if (node_to == graph.exit_node) {
        ending_angle = map.exit.orientation;
    }
    if (node_from == graph.robot_position) {
        starting_angle = map.robot_position.orientation;
    }
    ExecutableDubinsTrajectory output;
    bool success = find_optimal_trajectory(
        {graph.nodes[node_from].value, starting_angle},
        {graph.nodes[node_to].value, ending_angle},
        this->occupation_approximation,
        this->k,
        output,
        this->velocity
    );

    if (success) {
        this->nodes[node_from].add_connection(starting_angle, {
            &(this->nodes[node_from]),
            ending_angle,
            output
        });
    }
}


