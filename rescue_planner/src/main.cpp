#include <iostream>
#include "interface/debug_interface.hpp"
#include "interface/interface.hpp"
#include "graph_builder/combinatorial_graph_builder.hpp"
#include "util/display.hpp"


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
