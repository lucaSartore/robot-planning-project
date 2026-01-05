#pragma once
#include <vector>
#include "../interface/interface.hpp"
#include <variant>

enum DubinsTrajectoryKind {
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL
};

class Trajectory {
public:
    virtual Pose operator()(float time) const;
};

class RotatingTrajectory: public Trajectory {
public:
    Point center;
    float radius;
    float omega;
    float phi;
    virtual Pose operator()(float time) const;
    RotatingTrajectory(Point center, float radius, float omega, float phi);
    static RotatingTrajectory LTrajectory(Pose initial_pose, float radius, float speed);
    static RotatingTrajectory RTrajectory(Pose initial_pose, float radius, float speed);
};

class StraightTrajectory: public Trajectory {
public:
    Pose initial_pose;
    float speed;
    virtual Pose operator()(float time) const;
    StraightTrajectory(Pose initial_pose, float speed);
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
    void scale(float lambda);
};

class ExecutableDubinsTrajectory{
public:
    /// transition time between first and second trajectory
    float t1;
    /// transition time between second and fisrt trajectory
    float t2;
    /// total length of the trajectory
    float length;
    float time;
    variant<RotatingTrajectory,StraightTrajectory> trajectory_1 = StraightTrajectory({{0,0},0},0);
    variant<RotatingTrajectory,StraightTrajectory> trajectory_2 = StraightTrajectory({{0,0},0},0);
    variant<RotatingTrajectory,StraightTrajectory> trajectory_3 = StraightTrajectory({{0,0},0},0);

    Pose operator()(float time);
    explicit ExecutableDubinsTrajectory();
    ExecutableDubinsTrajectory(DubinsTrajectory trajectory, Pose start, float k, float v);
    vector<Pose> get_trajectory(int resolution = 10);
    void debug(int resolution = 10);
};

class OccupationApproximation {
    float x_min, x_max, y_min, y_max;
    int resolution;
    vector<vector<bool>> ocupations;
public:
    OccupationApproximation(Map const& map, int resolution=1000);
    bool is_available(Point point);
};

bool find_optimal_trajectory(Pose start, Pose end, OccupationApproximation const& occupation, float kmax, ExecutableDubinsTrajectory& output, int collision_resolution= 5);