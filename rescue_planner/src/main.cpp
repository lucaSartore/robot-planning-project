#include <ros/ros.h>
#include <iostream>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Pose.h"
#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "obstacles_msgs/ObstacleMsg.h"
#include <iostream>
#include "interface/ros_interface.hpp"

using namespace std;

void exitPointCallback(const geometry_msgs::PoseArray & msg) {
    auto pos = msg.poses[0];
    // note: orientation is also important for the gate
    ROS_INFO("Gate found at: x=[%f. y=[%f]", pos.position.x, pos.position.y);
}

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void mapCallback(const geometry_msgs::Polygon & msg) {
    ROS_INFO("I read a poligon with [%ld] points", msg.points.size());
}

void chatterCallback(const obstacles_msgs::ObstacleArrayMsg & msg) {
    ROS_INFO("I read [%ld] obstacles", msg.obstacles.size());
}

int main(int argc, char** argv){

    cout << "Done!" << endl;
    ros::init(argc, argv, "planner");
    
    RosInterface interface = RosInterface();

    // ros::NodeHandle n;
    
    // ros::Subscriber sub = n.subscribe("/obstacles", 1000, chatterCallback);
    // ros::Subscriber sub = n.subscribe("/victims", 1000, chatterCallback);
    // ros::Subscriber sub = n.subscribe("/map_borders", 1000, mapCallback);
    // ros::Subscriber sub = n.subscribe("/gates", 1000, exitPointCallback);
    
    ros::spin();
    // Obstacle obstacle;
    return 0;
}
