#pragma once
#include <vector>
#include "../interface/interface.hpp"

enum DubinsTrajectoryKind {
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL
};

class DubinsTrajectory {
public:
    DubinsTrajectoryKind kind;
    float t1;
    float t2;
    float t3;
    float total_length;
    DubinsTrajectory(DubinsTrajectoryKind kind, float t1, float t2, float t3);
    DubinsTrajectory();
};

class DubinsTrajectoryRaw {
public:
    vector<Point> trajectory;
    explicit DubinsTrajectoryRaw(vector<Point> trajectory);
    DubinsTrajectoryRaw(DubinsTrajectory trajectory, Pose start, float k, float v, int resolution = 100);
};

class OccupationApproximation {
    float x_min, x_max, y_min, y_max;
    int resolution;
    vector<vector<bool>> ocupations;
public:
    OccupationApproximation(Map const& map, int resolution=1000);
    bool is_available(Point point);
};

bool find_optimal_trajectory(Pose start, Pose end, OccupationApproximation const& occupation, float kmax, DubinsTrajectory& output);