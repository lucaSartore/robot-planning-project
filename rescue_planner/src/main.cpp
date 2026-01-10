#include <iostream>
#include <map>

#include "interface/debug_interface.hpp"
#include "interface/interface.hpp"
#include "graph_builder/combinatorial_graph_builder.hpp"
#include "util/display.hpp"
#include "trajectory_planner/trajectory_planner.hpp"
#include "dubins_graph/dubins_graph.hpp"
#include "graph_builder/sampling_graph_builder.hpp"
#include "util/constants.hpp"


#ifdef DOCKER_ROS
#include <ros/ros.h>
#include "interface/ros_interface.hpp"
#endif
using namespace std;



Result process_map(Map map, bool debug_best = false, float debug_graph = false, bool debug_refinement = false, bool print_times = false, float time_limit = 120) {
    std::chrono::steady_clock::time_point start_full, end_full;
    std::chrono::steady_clock::time_point start, end;
    auto print_time = [&](string name) {
        if (!print_times) return;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        cout << "execution time for " << name << " = " << ms << " [ms]" << endl;
    };

    start = std::chrono::steady_clock::now();
    start_full = std::chrono::steady_clock::now();
    OccupationApproximation occupation = {map, 1000, 0.5};
    end = std::chrono::steady_clock::now();
    print_time("occupation approximation creation");

    Graph graph= {{},0,0,{}};

    start = std::chrono::steady_clock::now();
    if (STRATEGY == COMBINATORIAL) {
        CombinatorialGraphBuilder builder = CombinatorialGraphBuilder();
        graph = builder.convert(map);
        graph.add_skip_ahead_connections();
        graph.add_skip_ahead_connections();
    } else {
        SamplingGraphBuilder builder = {occupation, N_SAMPLED_POINTS, N_NEAREST};
        graph = builder.convert(map);
        graph.add_skip_ahead_connections();
    }
    end = std::chrono::steady_clock::now();
    print_time("graph creation");

    if (debug_graph) {
        graph.debug();
    }

    start = std::chrono::steady_clock::now();
    auto dubins_graph = DubinsGraph(
        map,
        occupation,
        graph,
        VELOCITY,
        ROBOT_K
    );
    end = std::chrono::steady_clock::now();
    print_time("dubins graph creation");

    start = std::chrono::steady_clock::now();
    auto search = RescueOrderSearch(dubins_graph);
    search.execute();
    end = std::chrono::steady_clock::now();
    print_time("execution of search");


    start = std::chrono::steady_clock::now();
    Result best = {{},{},{}};
    if (debug_refinement) {
        best = search.get_best_solution(time_limit);
        cout << "time-score without refinement" << " = " << best.total_time << best.total_value << endl;
        search.refine_all_solutions(M_PI/4);
        best = search.get_best_solution(time_limit);
        cout << "time-score with one-step refinement" << " = " << best.total_time << best.total_value << endl;
        search.refine_all_solutions(M_PI/32);
        best = search.get_best_solution(time_limit);
        cout << "time-score with two-step refinement" << " = " << best.total_time << best.total_value << endl;
    } else {
        search.refine_all_solutions(M_PI/4);
        search.refine_all_solutions(M_PI/32);
        best = search.get_best_solution(time_limit);
    }
    end = std::chrono::steady_clock::now();
    print_time("solutiohn refinement");

    end_full = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_full - start_full).count();
    if (print_times) {
        cout << "total execution time" << " = " << ms << " [ms]" << endl;
    }

    cout << "best solution took " << best.total_time << " seconds and had a save-score of " << best.total_value << endl;

    if (debug_best) {
        search.debug(best);
    }
    return best;
}

#ifdef DOCKER_ROS
int main_ros(int argc, char** argv) {
    cout << "start" << endl;
    ros::init(argc, argv, "planner");
    RosInterface interface = RosInterface();

    Map map;
    while (true) {
        ros::spinOnce();

        auto map_optional = interface.TryGetMap();

        if (map_optional.has_value()) {
            map = map_optional.value();
            break;
        }
    }

    auto best =  process_map(
        map,
        true,
        false,
        true,
        true,
        120
    );
    interface.OutputTrajectory(best);
    ros::spin();

    return 0;
}
#else
int main_debug(int argc, char** argv) {
    DebugInterface interface = DebugInterface();
    auto map = interface.GetMap();
    auto _ =  process_map(
        map,
        false,
        false,
        true,
        true,
        120
        );
    return 0;
}
#endif

int main(int argc, char** argv){
#ifdef DOCKER_ROS
    return main_ros(argc, argv);
#else
    return main_debug(argc, argv);
#endif
}
