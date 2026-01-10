#pragma once
enum Strategy{ COMBINATORIAL, SAMPLING };

/// max velocity of the robot
const float VELOCITY = 1;
/// 1/robot turning radius
const float ROBOT_K = 1;
/// robot radius used for collision detection
const float ROBOT_RADIUS = 0.8;
/// resolution used in the occupation approximation
const int OCCUPATION_RESOLUTION_X = 1000;
/// strategy used for the motion planning graph
const Strategy STRATEGY  = COMBINATORIAL;
/// number of samles in the sampling based approach
const int N_SAMPLED_POINTS = 300;
/// number of nearest point to connect to in the sampling approach
const int N_NEAREST = 10;
/// thread to spawn for multithreaded workloads
const int NUM_WORKERS = 12;
