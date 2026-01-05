#include <iostream>
#include "interface/debug_interface.hpp"
#include "interface/interface.hpp"
#include "graph_builder/combinatorial_graph_builder.hpp"
#include "util/display.hpp"
#include "trajectory_planner/trajectory_planner.hpp"


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

    // CombinatorialGraphBuilder builder = CombinatorialGraphBuilder();
    // auto graph = builder.convert(map);
    // graph.debug();

    Pose start = {{-3, 3}, -1};
    Pose end = {{3, 7}, -2};
    float kmax = 2;
    float vmax = 1;
    ExecutableDubinsTrajectory trajectory = ExecutableDubinsTrajectory();
    OccupationApproximation occupation = {map, 1000, 0.5};

    // find_optimal_trajectory(
    //     start,
    //     end,
    //     {map, 1000, 0.5},
    //     kmax,
    //     trajectory,
    //     vmax
    // );
    // trajectory.debug();

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
