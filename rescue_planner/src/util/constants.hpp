#pragma once

/// max velocity of the robot
const float VELOCITY = 1.0;
/// 1/robot turning radius
const float ROBOT_K = 1.4;
/// robot radius used for collision detection
const float ROBOT_RADIUS = 0.5;
/// resolution used in the occupation approximation
const int OCCUPATION_RESOLUTION_X = 1000;
enum Strategy{ COMBINATORIAL, SAMPLING };
/// strategy used for the motion planning graph
const Strategy STRATEGY  = Strategy::SAMPLING;
