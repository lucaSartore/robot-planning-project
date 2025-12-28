#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"

#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "obstacles_msgs/ObstacleMsg.h"

// #include <iostream>
#include "../../map_pkg/include/map_pkg/obstacle_struct.hpp"

using namespace std;

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const obstacles_msgs::ObstacleArrayMsg & msg)
{
    cout << msg.obstacles.size() << endl;
    ROS_INFO("I read [%ld] obstacles", msg.obstacles.size());
}

int main(int argc, char** argv){

    ros::init(argc, argv, "planner");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/obstacles", 1000, chatterCallback);
    
    ros::spin();
    // Obstacle obstacle;
    return 0;
}
