#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"


#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Pose.h"

#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "obstacles_msgs/ObstacleMsg.h"

// #include <iostream>
#include "../../map_pkg/include/map_pkg/obstacle_struct.hpp"

using namespace std;


void exitPointCallback(const geometry_msgs::Polygon & msg) {
    ROS_INFO("I read a poligon with [%ld] points", msg.points.size());
}

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void mapCallback(const geometry_msgs::Polygon & msg) {
    ROS_INFO("I read a poligon with [%ld] points", msg.points.size());
}

void chatterCallback(const obstacles_msgs::ObstacleArrayMsg & msg) {
    ROS_INFO("I read [%ld] obstacles", msg.obstacles.size());
}

int main(int argc, char** argv){

    ros::init(argc, argv, "planner");
    
    ros::NodeHandle n;
    
    // ros::Subscriber sub = n.subscribe("/obstacles", 1000, chatterCallback);
    // ros::Subscriber sub = n.subscribe("/victims", 1000, chatterCallback);
    // ros::Subscriber sub = n.subscribe("/map_borders", 1000, mapCallback);
    ros::Subscriber sub = n.subscribe("/gates", 1000, exitPointCallback);
    
    ros::spin();
    // Obstacle obstacle;
    return 0;
}
