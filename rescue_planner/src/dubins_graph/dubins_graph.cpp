#include "dubins_graph.hpp"

#include <assert.h>
#include <math.h>
#include <thread>
#include <unordered_set>

#include "../trajectory_planner/trajectory_planner.hpp"
#include "../util/constants.hpp"


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

DubinsNode::DubinsNode(const DubinsNode &other) {
    id = other.id;
    connections = other.connections;
}

DubinsNode &DubinsNode::operator=(const DubinsNode &other) {
    this->id = other.id;
    this->connections = other.connections;
    return *this;
}

void DubinsNode::add_connection(float angle, DubinsEdge edge) {
    this->connections_mutex.lock();
    if (this->connections.find(angle) == this->connections.end()) {
        this->connections[angle] = {};
    }
    this->connections[angle].push_back(edge);
    this->connections_mutex.unlock();
}


DubinsGraph::DubinsGraph(Map& map, OccupationApproximation& occupation_approximation, Graph& graph, float velocity, float k, optional<Result> initial_guess, float initial_guess_search_range)
    :occupation_approximation(occupation_approximation), map(map), graph(graph) {
    this->velocity = velocity;
    this->k = k;
    this->nodes = {};
    this->initial_guess = initial_guess;
    this->initial_guess_search_range = initial_guess_search_range;
    if (initial_guess.has_value()) {
        this->insert_nodes_in_graph_with_initial_guess();
        this->generate_edges_with_initial_guess();
    } else {
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
}

void DubinsGraph::insert_nodes_in_graph() {
    for (const auto&[fst, snd]: this->graph.nodes) {
        auto id = fst;
        auto node = snd;
        this->nodes[id] = DubinsNode(id);
    }
}

void DubinsGraph::insert_nodes_in_graph_with_initial_guess() {
    assert(this->initial_guess.has_value());
    unordered_set<int> nodes_to_keep = {};
    for (auto  &n: this->initial_guess.value().nodes_order) {
        nodes_to_keep.insert(std::get<0>(n));
    }
    for (const auto&[fst, snd]: this->graph.nodes) {
        if (nodes_to_keep.find(fst) == nodes_to_keep.end()) {
            continue;
        }
        auto id = fst;
        auto node = snd;
        this->nodes[id] = DubinsNode(id);
    }

}

void DubinsGraph::generate_edges_with_initial_guess() {
    assert(this->initial_guess.has_value());

    auto path = initial_guess.value().nodes_order;

    auto generate_angles = [&](float starting) {
        vector<float> to_return = {};
        to_return.reserve(9);
        for (int i=-4; i<=4; i++) {
            to_return.push_back(
                starting + i / 4.0 * this->initial_guess_search_range
                );
        }
        return to_return;
    };

    for (int i=0; i<path.size()-1; i++) {
        auto start = path[i];
        auto end = path[i+1];
        auto start_id = std::get<0>(start);
        auto end_id = std::get<0>(end);
        auto start_angle = std::get<1>(start);
        auto end_angle = std::get<1>(end);

        auto start_angles = generate_angles(start_angle);
        auto end_angles = generate_angles(end_angle);

        for (float s: start_angles) {
            for (float e: end_angles) {
                generate_edge(start_id, end_id, s,e);
            }
        }
    }
}


void DubinsGraph::generate_edges(vector<float> const& angles) {
    int index = 0;
    mutex index_mutex;

    int initial_size = this->nodes.size();

    vector<int> nodes_indexes = {};
    nodes_indexes.reserve(this->nodes.size());
    for (auto &[_,node]: this->nodes) {
        nodes_indexes.push_back(node.id);
    }


    auto thread = [&]() {
        while (true) {
            index_mutex.lock();
            int local_index = index;
            index += 1;
            index_mutex.unlock();
            if (local_index >= this->nodes.size()) {
                break;
            }
            int id = nodes_indexes[local_index];
            generate_edge(id, angles);
        }

    };
    vector<std::thread> threads;

    for (int i=0; i<NUM_WORKERS; i++) {
        threads.push_back(std::thread(thread));
    }

    for (int i=0; i<NUM_WORKERS; i++) {
        threads[i].join();
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
            &(this->nodes[node_to]),
            ending_angle,
            output
        });
    }
}


