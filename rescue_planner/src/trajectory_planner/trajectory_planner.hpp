#pragma once
#include <vector>
#include "../interface/common_types.hpp"
#include <variant>

enum DubinsTrajectoryKind {
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL
};


class Velocities {
public:
    float velocity;
    float angular_velocity;
    Velocities(float velocity, float angular_velocity);
};

class Trajectory {
public:
    virtual Pose operator()(float time) const;
    virtual Velocities get_velocities(float time) const;
};

class RotatingTrajectory: public Trajectory {
public:
    Point center;
    float radius;
    float omega;
    float phi;
    virtual Pose operator()(float time) const;
    virtual Velocities get_velocities(float time) const;
    RotatingTrajectory(Point center, float radius, float omega, float phi);
    static RotatingTrajectory LTrajectory(Pose initial_pose, float radius, float speed);
    static RotatingTrajectory RTrajectory(Pose initial_pose, float radius, float speed);
};

class StraightTrajectory: public Trajectory {
public:
    Pose initial_pose;
    float speed;
    virtual Pose operator()(float time) const;
    virtual Velocities get_velocities(float time) const;
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
    std::variant<RotatingTrajectory,StraightTrajectory> trajectory_1 = StraightTrajectory({{0,0},0},0);
    std::variant<RotatingTrajectory,StraightTrajectory> trajectory_2 = StraightTrajectory({{0,0},0},0);
    std::variant<RotatingTrajectory,StraightTrajectory> trajectory_3 = StraightTrajectory({{0,0},0},0);

    Pose operator()(float time);
    Velocities get_velocities(float time) const;
    explicit ExecutableDubinsTrajectory();
    ExecutableDubinsTrajectory(DubinsTrajectory trajectory, Pose start, float k, float v);
    vector<Pose> get_trajectory(int resolution = 10);
    void debug(int resolution = 10);
};


class OccupationApproximation {
public:
    OccupationApproximation(Map const &map, int resolution_x, float radius, float margins = 0.25f);

    bool get(float x, float y) const;
    bool get(Point point) const;

    void set(float x, float y, bool value);
    void set(Point point, bool value);

    bool is_available(Point point) const;

    void debug(int scaling=10);

private:
    std::tuple<int, int> get_indexes(float x, float y) const;
    Point inverse(int x, int y) const;

    void draw_line(Point start, Point end);
    void draw_circle(Point center, float radius);
    void draw_map(Map const& map);

    static std::vector<std::vector<bool>> get_kernel(int radius);
    static std::vector<std::vector<bool>> dilation(const std::vector<std::vector<bool>>& source,
                                                   const std::vector<std::vector<bool>>& kernel);

    std::vector<std::vector<bool>> ocupations;

    float x_min, x_max;
    float y_min, y_max;

    int resolution_x;
    int resolution_y;
};

bool find_optimal_trajectory(Pose start, Pose end, OccupationApproximation const& occupation, float kmax, ExecutableDubinsTrajectory& output, float velocity, int collision_resolution= 5);