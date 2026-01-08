#include "dubins_graph.hpp"
#include <algorithm>
#include <execution>

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

    auto worker = [&](vector<int> order) {
        auto g = GraphSearch(graph, order);
        vector raw_result = g.execute();
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
            raw_result,
            victims
        );

        results_mutex.lock();
        results.push_back(result);
        results_mutex.unlock();
    };

    std::for_each(
        permutations.begin(),
        permutations.end(),
        worker
    );


}

Result::Result(vector<ExecutableDubinsTrajectory> trajectory, vector<tuple<int, Victim>> victims) {
    this->trajectory = trajectory;
    this->victims = victims;
    total_value = 0;
    for (auto v: victims) {
        total_value += std::get<1>(v).value;
    }
    total_length = 0;
    for (auto v: trajectory) {
        total_length += v.length;
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

Result RescueOrderSearch::get_best_solution(float time_limit) {
    float length_limit = time_limit * graph.velocity;
    vector<Result> filtered_results = {};
    for (auto &r: results) {
        if (r.total_length < length_limit) {
            filtered_results.push_back(r);
        }
    }

    if (filtered_results.empty()) {
        return {{},{}};
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

    return *std::min_element(filtered_results_2.begin(), filtered_results_2.end(), [](Result const& a, Result const& b) {
        return a.total_length < b.total_length;
    });
}
