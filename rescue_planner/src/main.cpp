#include <iostream>
#include "interface/debug_interface.hpp"
#include "interface/interface.hpp"
#include "graph_builder/combinatorial_graph_builder.hpp"
#include "util/display.hpp"
#include "trajectory_planner/trajectory_planner.hpp"
#include "dubins_graph/dubins_graph.hpp"
#include "util/constants.hpp"


#ifdef DOCKER_ROS
#include <ros/ros.h>
#include "interface/ros_interface.hpp"
#endif
using namespace std;



RescueOrderSearch process_map(Map map) {
    CombinatorialGraphBuilder builder = CombinatorialGraphBuilder();
    auto graph = builder.convert(map);
    // graph.debug();
    graph.add_skip_ahead_connections();
    graph.add_skip_ahead_connections();
    // graph.add_skip_ahead_connections();
    // graph.add_skip_ahead_connections();
    // graph.debug();

    OccupationApproximation occupation = {map, 1000, 0.5};
    // occupation.debug();

    auto dubins_graph = DubinsGraph(
        map,
        occupation,
        graph,
        VELOCITY,
        ROBOT_K
    );

    auto search = RescueOrderSearch(dubins_graph);
    search.execute();
    return search;
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

    auto search = process_map(map);
    auto best = search.get_best_solution(50);
    interface.OutputTrajectory(best);
    ros::spin();

    return 0;
}
#else
int main_debug(int argc, char** argv) {
    DebugInterface interface = DebugInterface();
    auto map = interface.GetMap();
    auto search =  process_map(map);
    auto best = search.get_best_solution(100);

    vector<Point> to_display = {};
    vector<tuple<Point,Point>> lines = {};
    for (int i=0; i<50; i++) {
        float t = best.total_time * i / 50.0;
        auto p = best.get_at(t);
        auto pose = std::get<0>(p);
        auto v = std::get<1>(p);
        auto p1 = pose.position;
        auto p2 = pose.position + Point::FromPolar(pose.orientation, v.velocity);
        lines.push_back({p1,p2});
        to_display.push_back(p1);
    }
    display(lines, to_display);

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
