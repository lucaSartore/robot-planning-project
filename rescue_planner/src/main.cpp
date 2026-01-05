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
    // cout << map << endl;

    CombinatorialGraphBuilder builder = CombinatorialGraphBuilder();
    auto graph = builder.convert(map);
    graph.debug();
    graph.add_skip_ahead_connections();
    graph.add_skip_ahead_connections();
    graph.debug();

    ExecutableDubinsTrajectory trajectory = ExecutableDubinsTrajectory();
    OccupationApproximation occupation = {map, 1000, 0.5};
    //occupation.debug()

    auto dubins_graph = DubinsGraph(
        map,
        occupation,
        graph,
        VELOCITY,
        ROBOT_K
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
