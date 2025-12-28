#include "ros_interface.hpp"
#include "interface.hpp"


Map RosInterface::built_map;
Map RosInterface::under_construction_map;
bool RosInterface::initialized;
bool RosInterface::obstacles_done;
bool RosInterface::victims_done;
bool RosInterface::map_done;
bool RosInterface::exit_done;
std::mutex RosInterface::map_ready_mutex;
std::mutex RosInterface::edit_mutex;
std::optional<ros::NodeHandle> RosInterface::node_handle;
std::vector<ros::Subscriber> RosInterface::subscribers;

RosInterface::RosInterface(){
    RosInterface::edit_mutex.lock();

    if (RosInterface::initialized == true) {
        return;
    }
    RosInterface::initialized = true;
    std::cout << "Entering interface config" << std::endl;

    RosInterface::built_map = Map();
    RosInterface::under_construction_map = Map();
    RosInterface::obstacles_done = false;
    RosInterface::victims_done = false;
    RosInterface::map_done = false;
    RosInterface::exit_done = false;
    // mutex should be initialized automatically with default constructor
    // RosInterface::map_ready_mutex = std::mutex();
    // RosInterface::edit_mutex = std::mutex();
    RosInterface::node_handle = ros::NodeHandle();
    RosInterface::subscribers = std::vector<ros::Subscriber>();

    // this is lock and will be unlocked when the map is initialized
    // for the first time
    RosInterface::map_ready_mutex.lock();


    RosInterface::subscribers.push_back(
        RosInterface::node_handle.value().subscribe("/obstacles", 1, this->ObstacleCallback)
    );
    RosInterface::subscribers.push_back(
        RosInterface::node_handle.value().subscribe("/victims", 1, this->VictimCallback)
    );
    RosInterface::subscribers.push_back(
        RosInterface::node_handle.value().subscribe("/map_borders", 1, this->MapCallback)
    );
    RosInterface::subscribers.push_back(
        RosInterface::node_handle.value().subscribe("/gates", 1, this->ExitCallback)
    );


    RosInterface::edit_mutex.unlock();

}


void RosInterface::ObstacleCallback(const obstacles_msgs::ObstacleArrayMsg & msg) {
    ROS_INFO("I read [%ld] obstacles", msg.obstacles.size());
}

void RosInterface::VictimCallback(const obstacles_msgs::ObstacleArrayMsg & msg) {
    ROS_INFO("I read [%ld] obstacles", msg.obstacles.size());
}

void RosInterface::MapCallback(const geometry_msgs::Polygon & msg) {
    ROS_INFO("I read a poligon with [%ld] points", msg.points.size());
}

void RosInterface::ExitCallback(const geometry_msgs::PoseArray & msg) {
    auto pos = msg.poses[0];
    ROS_INFO("Gate found at: x=[%f. y=[%f]", pos.position.x, pos.position.y);
}

void RosInterface::TryExportMap() {
    RosInterface::edit_mutex.lock();

    bool is_ready = RosInterface::obstacles_done && RosInterface::victims_done
        && RosInterface::map_done && RosInterface::exit_done;

    RosInterface::built_map = RosInterface::under_construction_map;
    RosInterface::under_construction_map = Map();

    RosInterface::obstacles_done = false;
    RosInterface::victims_done = false;
    RosInterface::map_done = false;
    RosInterface::exit_done = false;

    RosInterface::edit_mutex.unlock();
}

Map RosInterface::GetMap(){
    // making sure that the map is done initializing
    RosInterface::map_ready_mutex.lock();
    RosInterface::map_ready_mutex.unlock();

    RosInterface::edit_mutex.lock();
    Map to_return = RosInterface::built_map;
    RosInterface::edit_mutex.unlock();
    return to_return;
}

void RosInterface::OutputTrajectory(vector<Point> trajectory) {
}
