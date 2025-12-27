#include <ros/ros.h>
#include "std_msgs/String.h"
// #include <iostream>
#include "../../map_pkg/include/map_pkg/obstacle_struct.hpp"

using namespace std;

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("one message");
}

int main(int argc, char** argv){

    ros::init(argc, argv, "planner");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("obstacles", 1000, chatterCallback);
    
    // Obstacle obstacle;
    return 0;
}
