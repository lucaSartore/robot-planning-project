#include "ros_interface.hpp"
#include "interface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Pose.h"


Map RosInterface::built_map;
Map RosInterface::under_construction_map;
bool RosInterface::initialized;
bool RosInterface::obstacles_done;
bool RosInterface::victims_done;
bool RosInterface::map_done;
bool RosInterface::exit_done;
bool RosInterface::position_done;
std::mutex RosInterface::map_ready_mutex;
std::mutex RosInterface::edit_mutex;
std::optional<ros::NodeHandle> RosInterface::node_handle;
std::vector<ros::Subscriber> RosInterface::subscribers;

Pose create_pose(geometry_msgs::Pose pos) {

    tf2::Quaternion q_tf2;
    tf2::fromMsg(pos.orientation, q_tf2);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);

    return  Pose(
        Point(pos.position.x, pos.position.y),
        yaw
    );
}


RosInterface::RosInterface(){
    RosInterface::edit_mutex.lock();

    if (RosInterface::initialized == true) {
        return;
    }
    RosInterface::initialized = true;

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
    RosInterface::subscribers.push_back(
        RosInterface::node_handle.value().subscribe("/limo0/odom", 1, this->RobotPositionCallback)
    );


    RosInterface::edit_mutex.unlock();

}


void RosInterface::ObstacleCallback(const obstacles_msgs::ObstacleArrayMsg & msg) {
    RosInterface::edit_mutex.lock();
    auto v = vector<Obstacle>();
    for (int i=0; i<msg.obstacles.size(); i++) {
        auto obstacle = msg.obstacles[i];

        if (obstacle.radius == 0) {
            auto v2 = vector<Point>();
            
            for (int j=0; j<obstacle.polygon.points.size(); j++) {
                auto point = obstacle.polygon.points[j];
                v2.push_back(
                    Point(point.x, point.y)
                );
            }

            v.push_back(Obstacle::CreatePolygon(v2));
        } else {
            v.push_back(Obstacle::CreateCylinder(
                Point(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y),
                obstacle.radius 
            ));
        }
    }
    RosInterface::under_construction_map.obstacles = v;
    RosInterface::obstacles_done = true;
    RosInterface::edit_mutex.unlock();

    TryExportMap();
}

void RosInterface::VictimCallback(const obstacles_msgs::ObstacleArrayMsg & msg) {
    RosInterface::edit_mutex.lock();
    auto v = vector<Victim>();
    for (int i=0; i<msg.obstacles.size(); i++) {
        auto obstacle = msg.obstacles[i];
        v.push_back(Victim(
            Point(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y),
            obstacle.radius 
        ));
    }
    RosInterface::under_construction_map.victims = v;
    RosInterface::victims_done = true;
    RosInterface::edit_mutex.unlock();

    TryExportMap();
}

void RosInterface::MapCallback(const geometry_msgs::Polygon & msg) {
    RosInterface::edit_mutex.lock();
    auto v = vector<Point>();
    for (int i=0; i<msg.points.size(); i++) {
        auto point = msg.points[i];
        v.push_back(Point(point.x, point.y));
    }
    RosInterface::under_construction_map.borders = v;
    RosInterface::map_done = true;
    RosInterface::edit_mutex.unlock();

    TryExportMap();
}

void RosInterface::ExitCallback(const geometry_msgs::PoseArray & msg) {
    auto pos = create_pose(msg.poses[0]);

    RosInterface::edit_mutex.lock();
    RosInterface::under_construction_map.exit = pos;
    RosInterface::exit_done = true;
    RosInterface::edit_mutex.unlock();

    TryExportMap();
}


void RosInterface::RobotPositionCallback(const nav_msgs::Odometry & msg) {
    auto pos = create_pose(msg.pose.pose);

    RosInterface::edit_mutex.lock();
    RosInterface::under_construction_map.robot_position = pos;
    RosInterface::position_done = true;
    RosInterface::edit_mutex.unlock();

    TryExportMap();
}

void RosInterface::TryExportMap() {
    RosInterface::edit_mutex.lock();

    bool is_ready = RosInterface::obstacles_done && RosInterface::victims_done
        && RosInterface::map_done && RosInterface::exit_done && RosInterface::position_done;

    if (!is_ready) {
        RosInterface::edit_mutex.unlock();
        return;
    }

    RosInterface::built_map = RosInterface::under_construction_map;
    RosInterface::under_construction_map = Map();

    RosInterface::obstacles_done = false;
    RosInterface::victims_done = false;
    RosInterface::map_done = false;
    RosInterface::exit_done = false;
    RosInterface::position_done = false;

    RosInterface::edit_mutex.unlock();

    cout << "Map build: " << RosInterface::built_map << endl;
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

void RosInterface::OutputTrajectory(vector<Pose> trajectory) {
    for
}
