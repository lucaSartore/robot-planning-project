#include <iostream>
#include "interface/debug_interface.hpp"
#include "interface/interface.hpp"
#include "graph_builder/combinatorial_graph_builder.hpp"
#include "util/display.hpp"
#include "trajectory_planner/trajectory_planner.hpp"
#include "dubins_graph/dubins_graph.hpp"
#include "util/constants.hpp"


using namespace std;

#ifdef DOCKER_ROS
int main_ros(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    RosInterface interface = RosInterface();
    ros::spin();
    return 0;
}
#else
int main_debug(int argc, char** argv) {
    DebugInterface interface = DebugInterface();
    auto map = interface.GetMap();
    // map.robot_position = {{-1,-4},-1.5};
    // cout << map << endl;

    // map.obstacles.push_back(Obstacle::CreateCylinder({3,3}, 2));
    // display(map.get_obstacle_lines(), {});

    CombinatorialGraphBuilder builder = CombinatorialGraphBuilder();
    auto graph = builder.convert(map);
    // graph.debug();
    graph.add_skip_ahead_connections();
    graph.add_skip_ahead_connections();
    // graph.debug();

    ExecutableDubinsTrajectory trajectory = ExecutableDubinsTrajectory();
    OccupationApproximation occupation = {map, 1000, 0.5};
    // occupation.debug();

    auto dubins_graph = DubinsGraph(
        map,
        occupation,
        graph,
        VELOCITY,
        ROBOT_K
    );

    auto x = RescueOrderSearch(dubins_graph);
    // victims debug: 73, 74, 75 76
    // auto search = GraphSearch(dubins_graph, graph.victims_odes);
    // auto search = GraphSearch(dubins_graph, {56});
    // auto search = GraphSearch(dubins_graph, {});
    auto search = RescueOrderSearch(dubins_graph);
    search.execute();
    auto best = search.get_best_solution(80);
    search.debug(best);

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
