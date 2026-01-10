#include "dubins_graph.hpp"
#include <algorithm>
#include <execution>
#include <thread>

#include "../util/constants.hpp"

void print_vector(vector<int> v) {
    cout << "[";
    for (auto e: v) {
        cout << e << ", ";
    }
    cout << "]" << endl;
}

vector<vector<int>> all_permutations_with_excluded_elements(vector<int> options) {
    vector<vector<int>> result_combinations;
    vector<bool> present;
    present.reserve(options.size());
    for (int i = 0; i < options.size(); i++) {
        present.push_back(false);
    }

    int i;
    while (true) {
        vector<int> combination = {};
        // insert the boolean combination
        for (i = 0; i < options.size(); i++) {
            if (present[i]) {
                combination.push_back(options[i]);
            }
        }
        result_combinations.push_back(combination);


        // add one to boolean array (with carry on)
        for (i = 0; i < options.size(); i++) {
            present[i] = !present[i];
            // element was zero... no carry
            if (present[i]) {
                break;
            }
        }

        // full overflow;
        if (i == options.size()) {
            break;
        }
    }

    vector<vector<int>> result_permutations;
    for (auto combination: result_combinations) {
        do {
            result_permutations.push_back(combination);
        } while (std::next_permutation(combination.begin(), combination.end()));
    }

    return result_permutations;
}

RescueOrderSearch::RescueOrderSearch(DubinsGraph &graph): graph(graph) {
}


void RescueOrderSearch::debug(Result r) {
    auto g = GraphSearch(graph, {});
    g.debug(r.trajectory);
}

void RescueOrderSearch::execute() {

    auto victims = graph.graph.victims_odes;
    auto permutations = all_permutations_with_excluded_elements(victims);

    mutex results_mutex;

    auto execute_one = [&](int index) {
        auto order = permutations[index];
        auto g = GraphSearch(graph, order);
        auto raw_result = g.execute();
        vector<tuple<int,Victim>> victims;
        for (int v: order) {
            victims.push_back({
                v,
                *std::find_if(graph.map.victims.begin(), graph.map.victims.end(), [&](Victim x) {
                    return x.position == graph.graph.nodes[v].value;
                })
            });
        }
        auto result = Result(
            std::get<1>(raw_result),
            victims,
            std::get<0>(raw_result)
        );

        results_mutex.lock();
        results.push_back(result);
        results_mutex.unlock();
    };

    mutex counter_mutex;
    int counter = 0;

    auto thread = [&]() {
        while (true) {
            results_mutex.lock();
            int local_counter = counter;
            counter += 1;
            results_mutex.unlock();

            if (local_counter >= permutations.size()) {
                break;
            }

            execute_one(local_counter);
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

Result::Result(vector<ExecutableDubinsTrajectory> trajectory, vector<tuple<int, Victim>> victims, vector<tuple<int, float>> nodes_order) {
    this->trajectory = trajectory;
    this->victims = victims;
    this->nodes_order = std::move(nodes_order);
    total_value = 0;
    for (auto v: victims) {
        total_value += std::get<1>(v).value;
    }
    total_length = 0;
    total_time = 0;
    for (auto v: trajectory) {
        total_length += v.length;
        total_time += v.time;
    }
}

vector<Pose> Result::get_full_trajectory(int resolution) {
    vector<Pose> result;
    for (auto v: trajectory) {
        auto t = v.get_trajectory(resolution);
        result.insert(result.end(), t.begin(), t.end());
    }
    return result;
}

tuple<Pose, Velocities> Result::get_at(float time) {

    if (time >= total_time) {
        auto t = trajectory[trajectory.size() - 1];
        auto p = t(t.time);
        return {p,{0,0}};
    }

    float time_so_far = 0;
    int i = 0;
    while (time_so_far + trajectory[i].time < time) {
        time_so_far += trajectory[i].time;
        i += 1;
    }

    time -= time_so_far;
    auto p = trajectory[i](time);
    auto v = trajectory[i].get_velocities(time);
    return {p,v};
}


Result RescueOrderSearch::get_best_solution(float time_limit, bool refinement) {
    float length_limit = time_limit * VELOCITY;
    vector<Result> filtered_results = {};
    for (auto &r: results) {
        if (r.total_length < length_limit) {
            filtered_results.push_back(r);
        }
    }

    if (filtered_results.empty()) {
        return {{},{},{}};
    }

    auto max_value = std::max_element(filtered_results.begin(), filtered_results.end(), [](Result const& a, Result const& b) {
        return a.total_length < b.total_length;
    })->total_value;

    vector<Result> filtered_results_2 = {};

    for (auto &r: filtered_results) {
        if (r.total_value == max_value) {
            filtered_results_2.push_back(r);
        }
    }

    auto best = *std::min_element(filtered_results_2.begin(), filtered_results_2.end(), [](Result const& a, Result const& b) {
        return a.total_length < b.total_length;
    });

    if (!refinement) {
        return best;
    }

    auto g = DubinsGraph(
        this->graph.map,
        this->graph.occupation_approximation,
        this->graph.graph,
        this->graph.velocity,
        this->graph.k,
        {best}
    );

    // passing the victim orders is not necessary, as there is only one path
    auto gs = GraphSearch(g, {});
    auto raw_result = gs.execute();
    return Result(
        std::get<1>(raw_result),
        best.victims,
        std::get<0>(raw_result)
    );
}
