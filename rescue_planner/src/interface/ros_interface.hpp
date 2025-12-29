#pragma once

#include "interface.hpp"
#include <ros/ros.h>
#include <vector>
#include <mutex>
#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include <optional>

class RosInterface: public Interface {
public:
    RosInterface();
    virtual Map GetMap();
    virtual void OutputTrajectory(vector<Point> trajectory);

    // all member are static because there can be only one map at the same time
    // (due to the fact that only one topic of each kind exist)
    static Map built_map;
    static Map under_construction_map;
    static bool initialized;
    static bool obstacles_done;
    static bool victims_done;
    static bool map_done;
    static bool exit_done;
    static bool position_done;
    static std::mutex map_ready_mutex;
    static std::mutex edit_mutex;
    static std::optional<ros::NodeHandle> node_handle;
    static std::vector<ros::Subscriber> subscribers;
    static void TryExportMap();
    static void ObstacleCallback(const obstacles_msgs::ObstacleArrayMsg & msg);
    static void VictimCallback(const obstacles_msgs::ObstacleArrayMsg & msg);
    static void MapCallback(const geometry_msgs::Polygon & msg);
    static void ExitCallback(const geometry_msgs::PoseArray & msg);
    static void RobotPositionCallback(const nav_msgs::Odometry & msg);
};
