#include <assert.h>

#include "trajectory_planner.hpp"
#include "math.h"

#include <iostream>
#include <memory>

Pose direct_transformation(Pose p, float phi, float lambda, float x_ref, float y_ref);

Pose inverse_transfoformation(Pose p, float phi, float lambda, float x_ref, float y_ref);



bool find_optimal_trajectory(Pose start, Pose end, OccupationApproximation const& occupation, float kmax, DubinsTrajectory& output) {
    float dx = end.position.x - start.position.x;
    float dy = end.position.y - start.position.y;
    float phi = atan2(dy,dx);
    float lambda = 0.5 * sqrt(dx*dx + dy*dy);
    float x_ref = start.position.x * cos(phi) + start.position.y * sin(phi) + lambda;
    float y_ref = -start.position.x * sin(phi) + start.position.y * cos(phi);

    float new_k_max = lambda * kmax;

    cout << direct_transformation(start, phi, lambda, x_ref, y_ref) << endl;
    cout << direct_transformation(end, phi, lambda, x_ref, y_ref) << endl;
}

float mod2pi(float x) {
    float const two_pi = 2 * M_PI;
    while (x >= two_pi) x -= two_pi;
    while (x < 0) x += two_pi;
    return x;
}

DubinsTrajectory solve_lsl(float theta_start, float theta_end, float k) {
    float c = cos(theta_start) - cos(theta_end);
    float s = 2*k +  sin(theta_start) - sin(theta_end);
    float s1 = 1/k * mod2pi(atan2(c,s) - theta_start);
    float s2 = 1/k * sqrt(2 + 4*k*k - 2*cos(theta_start-theta_end) + 4*k*(sin(theta_start)-sin(theta_end)));
    float s3 = 1/k * mod2pi(theta_end - atan2(c,s));
    return {LSL, s1, s2, s3};
}

Pose direct_transformation(Pose p, float phi, float lambda, float x_ref, float y_ref) {
    float x = 1/lambda * (p.position.x * cos(phi) + p.position.y * sin(phi) - x_ref);
    float y = 1/lambda * (p.position.x * -sin(phi) + p.position.y * cos(phi) - y_ref);
    float angle = p.orientation - phi;
    return {{x,y}, angle};
}

Pose inverse_transfoformation(Pose p, float phi, float lambda, float x_ref, float y_ref) {
}


DubinsTrajectory::DubinsTrajectory(DubinsTrajectoryKind kind, float t1, float t2, float t3) {
    this->kind = kind;
    this->t1 = t1;
    this->t2 = t2;
    this->t3 = t3;
    this->total_length = t1 + t2 + t3;
}

DubinsTrajectory::DubinsTrajectory() {
    this->kind = DubinsTrajectoryKind::LRL;
    this->t1 = 0;
    this->t2 = 0;
    this->t3 = 0;
    this->total_length = 0;
}

DubinsTrajectoryRaw::DubinsTrajectoryRaw(vector<Point> trajectory, float duration) {
    this->trajectory = trajectory;
    this->duration = duration;
}

enum SingleTrajectoryKind {
    R,
    L,
    S
};

SingleTrajectoryKind slice_trajectory(DubinsTrajectoryKind t, int index) {
    assert(index >= 0);
    assert(index <= 2);
    tuple<SingleTrajectoryKind, SingleTrajectoryKind, SingleTrajectoryKind> result;
    switch (t) {
        case LSL:
            result = {L,S,L};
            break;
        case RSR:
            result = {R,S,R};
            break;
        case LSR:
            result = {L,S,R};
            break;
        case RSL:
            result = {R,S,L};
            break;
        case RLR:
            result = {R,L,R};
            break;
        case LRL:
            result = {L,R,L};
            break;
        default:
            assert(false);
    }
    switch (index) {
        case 0:
            return std::get<0>(result);
        case 1:
            return std::get<1>(result);
        case 2:
            return std::get<2>(result);
        default:
            assert(false);
    }
}



class Trajectory {
public:
    virtual Point operator()(float time) const {
        throw logic_error("not implemented");
    }
};

class RotatingTrajectory: public Trajectory {
public:
    Point center;
    float radius;
    float omega;
    float phi;
    virtual Point operator()(float time) const{
        return center + Point::FromPolar(
            phi + time * omega,
            radius
        );
    }
    RotatingTrajectory(Point center, float radius, float omega, float phi) {
        this->center = center;
        this->radius = radius;
        this->omega = omega;
        this->phi = phi;
    }
    static RotatingTrajectory LTrajectory(Pose initial_pose, float radius, float speed) {
        Point center = initial_pose.position + Point::FromPolar(initial_pose.orientation + M_PI/2, radius);
        float phi = atan2(initial_pose.position.y - center.y, initial_pose.position.x - center.x);
        float omega = speed / (M_PI * radius);
        return RotatingTrajectory(center, radius, omega, phi);
    }
    static RotatingTrajectory RTrajectory(Pose initial_pose, float radius, float speed) {
        Point center = initial_pose.position + Point::FromPolar(initial_pose.orientation - M_PI/2, radius);
        float phi = atan2(initial_pose.position.y - center.y, initial_pose.position.x - center.x);
        float omega = - speed / (M_PI * radius);
        return RotatingTrajectory(center, radius, omega, phi);
    }
};

class StraightTrajectory: public Trajectory {
public:
    Pose initial_pose;
    float speed;
    virtual Point operator()(float time) const {
        return initial_pose.position + Point::FromPolar(initial_pose.orientation, speed * time);
    }
    StraightTrajectory(Pose initial_pose, float speed) {
        this->initial_pose = initial_pose;
        this->speed = speed;
    }
};

unique_ptr<Trajectory> build_trajectory(SingleTrajectoryKind trajectory_kind, float k, float v, Pose start) {
    switch (trajectory_kind) {
        case L:
            return std::make_unique<RotatingTrajectory>(RotatingTrajectory::LTrajectory(start, k, v));
        case R:
            return std::make_unique<RotatingTrajectory>(RotatingTrajectory::RTrajectory(start, k, v));
        case S:
            return std::make_unique<StraightTrajectory>(StraightTrajectory(start, v));
        default:
            assert(false);
    }
}
DubinsTrajectoryRaw::DubinsTrajectoryRaw(DubinsTrajectory trajectory, Pose start, float k, float v, int resolution) {
    // we remove one element to keep it as the "start" of the sequence
    resolution -= 1;
    int c1 = round(resolution * trajectory.t1 / trajectory.total_length);
    int c2 = round(resolution * trajectory.t2 / trajectory.total_length);
    int c3 = resolution - c1 - c2;
    this->trajectory = {};
    this->trajectory.reserve(resolution);
    this->trajectory.push_back(start.position);

    auto kind_t1 = slice_trajectory(trajectory.kind, 0);
    auto kind_t2 = slice_trajectory(trajectory.kind, 1);
    auto kind_t3 = slice_trajectory(trajectory.kind, 2);
    Point end_1, end_2;
    unique_ptr<Trajectory> t1, t2, t3;

    auto process_trajectory = [&](unique_ptr<Trajectory> & t, int steps, float start_time, float duration) {
        for (int i=1; i<=steps; i++) {
            float time = i * duration / steps + start_time;
            this->trajectory.push_back(
                (*t)(time)
            );
        }
    };

    // fist trajectory
    t1 = build_trajectory(kind_t1, k, v, start);
    process_trajectory(t1, c1, 0, trajectory.t1);
    end_1 = this->trajectory.back();

    // second trajectory
    t2 = build_trajectory(kind_t2, k, v, end_1);
    process_trajectory(t2, c2, trajectory.t1, trajectory.t2);
    end_2 = this->trajectory.back();

    // third trajectory
    t2 = build_trajectory(kind_t3, k, v, end_2);
    process_trajectory(t3, c3, trajectory.t1 + trajectory.t2, trajectory.t3);

    this ->duration = trajectory.total_length;
}

