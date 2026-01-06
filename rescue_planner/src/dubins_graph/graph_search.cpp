#include <algorithm>

#include "dubins_graph.hpp"
#include "../trajectory_planner/trajectory_planner.hpp"
#include <unordered_map>
#include <algorithm>
#include <assert.h>
#include <set>
#include <unordered_set>
#include "../triangulation/triangulation.hpp"
#include "../util/display.hpp"
#include "../util/hashable_tuple.hpp"



/// class used in the priority queue
/// during the graph search
class ProgressPoint {
public:
    DubinsNode * node;
    float angle;
    float length_so_far;
    float total_heuristic_cost;
    vector<int> objectives;
    tuple<int, float, int> node_from;

    ProgressPoint(DubinsNode * node, float angle, float length_so_far, float total_heuristic_cost, vector<int> objectives, tuple<int, float, int> node_from) {
        this->node = node;
        this->angle = angle;
        this->length_so_far = length_so_far;
        this->total_heuristic_cost = total_heuristic_cost;
        this->objectives = std::move(objectives);
        this->node_from = node_from;
    }

    bool operator<(ProgressPoint const& other) const {
        return total_heuristic_cost < other.total_heuristic_cost;
    }
};


float GraphSearch::heuristic_cost(int node_start, int next_objective) {
    return distance(
        graph.graph.nodes[node_start].value,
        graph.graph.nodes[next_objective].value
    ) + heuristic_costs[next_objective];
}

GraphSearch::GraphSearch(DubinsGraph &graph, vector<int> victims): graph(graph) {
    start = &graph.nodes[graph.graph.robot_position];
    to_visit = {};
    for (int v: victims) {
        to_visit.push_back(&graph.nodes[v]);
    }
    to_visit.push_back(&graph.nodes[graph.graph.exit_node]);

    heuristic_costs = {};
    heuristic_costs[graph.graph.exit_node] = 0;

    // reverse the victims to calculate heuristic cost backword
    int v_prev = graph.graph.exit_node;
    std::reverse(victims.begin(), victims.end());
    for (int v: victims) {
        heuristic_costs[v] = heuristic_cost(v, v_prev);
        v_prev = v;
    }
}

vector<ExecutableDubinsTrajectory> GraphSearch::build_solution(unordered_map<tuple<int, float,int>, tuple<int, float, int>, hash_triplet> & backtracking, tuple<int, float, int> end) {
    vector<ExecutableDubinsTrajectory> to_return = {};
    auto current = end;
    while (true) {
        auto next = backtracking[current];
        if (std::get<0>(next) == -1) {
            std::reverse(to_return.begin(), to_return.end());
            return to_return;
        }
        int node_from_id = std::get<0>(next);
        float node_from_angle = std::get<1>(next);

        int node_to_id = std::get<0>(current);
        float node_to_angle = std::get<1>(current);

        auto node_from = graph.nodes[node_from_id];
        auto edges = node_from.connections[node_from_angle];
        auto connection = std::find_if(edges.begin(), edges.end(), [&](DubinsEdge e) {
            return e.arriving_angle == node_to_angle && e.node->id == node_to_id;
        });
        assert(connection != edges.end());
        to_return.push_back(connection->trajectory);

        current = next;
    }
}

vector<ExecutableDubinsTrajectory> GraphSearch::execute() {
    // set of nodes visited containing:
    // <node_id, angle, next_objectove>
    unordered_set<tuple<int, float, int>, hash_triplet> visited;

    vector<int> objectives = {};
    objectives.reserve(to_visit.size());
    for (auto v: to_visit) {
        objectives.push_back(v->id);
    }

    set<ProgressPoint> nodes_queue = {ProgressPoint(
        start,
        graph.map.robot_position.orientation,
        0,
        heuristic_cost(graph.graph.robot_position, to_visit[0]->id),
        objectives,
        {-1,-1,-1}
    )};

    unordered_map<tuple<int, float, int>, tuple<int, float, int>, hash_triplet> backtracking = {} ;

    while (!nodes_queue.empty()) {
        auto to_expand = *nodes_queue.begin();
        nodes_queue.erase(nodes_queue.begin());

        bool is_next_objective = to_expand.node->id == to_expand.objectives[0];

        if (!is_next_objective) {
            bool is_other_objective = find(
                to_expand.objectives.begin(),
                to_expand.objectives.end(),
                to_expand.node->id
            ) != to_expand.objectives.end();
            // we want to force the correct order
            if (is_other_objective) {
                continue;
            }
        }

        tuple<int, float, int> node_identifier = {to_expand.node->id, to_expand.angle, to_expand.objectives[0]};

        // node already visited... don't do anything
        if (visited.find(node_identifier) != visited.end()) {
            continue;
        }

        visited.insert(node_identifier);
        backtracking.insert({node_identifier, to_expand.node_from});

        // creating the new objectives list
        vector<int> new_objectives = vector<int>(to_expand.objectives.begin(), to_expand.objectives.end());
        if (is_next_objective) {
            new_objectives.erase(new_objectives.begin());
        }

        // solution found!
        if (new_objectives.empty()) {
            std::cout << "found solution with final cost: " << to_expand.length_so_far << std::endl;
            return build_solution(backtracking, node_identifier);
        }

        for (auto adj: to_expand.node->connections[to_expand.angle]) {
            float length_so_far = to_expand.length_so_far + adj.trajectory.length;
            nodes_queue.insert(ProgressPoint(
                adj.node,
                adj.arriving_angle,
                length_so_far,
                length_so_far + heuristic_cost(adj.node->id, new_objectives[0]),
                new_objectives,
                node_identifier
            ));
        }
    }

    return {};
}

void GraphSearch::debug(vector<ExecutableDubinsTrajectory> path) {
    auto obstacle_Lines = graph.map.get_obstacle_lines();
    auto path_points = vector<Point>();
    for (auto segment: path) {
        auto segment_points = segment.get_trajectory(3);
        for (auto p: segment_points) {
            path_points.push_back(p.position);
        }
    }
    display(obstacle_Lines, path_points);
}

