#include <algorithm>
#include <assert.h>
#include "trajectory_planner.hpp"
#include "math.h"
#include <iostream>
#include <memory>
#include "../util/display.hpp"

Pose direct_transformation(Pose p, float phi, float lambda, float x_ref, float y_ref);
DubinsTrajectory solve_lsl(float theta_start, float theta_end, float k);
DubinsTrajectory solve_rsr(float theta_start, float theta_end, float k);
DubinsTrajectory solve_lsr(float theta_start, float theta_end, float k);
DubinsTrajectory solve_rsl(float theta_start, float theta_end, float k);
DubinsTrajectory solve_lrl(float theta_start, float theta_end, float k);
DubinsTrajectory solve_rlr(float theta_start, float theta_end, float k);
DubinsTrajectory solve(DubinsTrajectoryKind kind, float theta_start, float theta_end, float k);




bool find_optimal_trajectory(Pose start, Pose end, OccupationApproximation const& occupation, float kmax, ExecutableDubinsTrajectory& output, float velocity, int collision_resolution) {
    float dx = end.position.x - start.position.x;
    float dy = end.position.y - start.position.y;
    float phi = atan2(dy,dx);
    float lambda = 0.5 * sqrt(dx*dx + dy*dy);
    float x_ref = start.position.x * cos(phi) + start.position.y * sin(phi) + lambda;
    float y_ref = -start.position.x * sin(phi) + start.position.y * cos(phi);

    float new_k_max = lambda * kmax;

    auto transformed_start =  direct_transformation(start, phi, lambda, x_ref, y_ref);
    auto transformed_end = direct_transformation(end, phi, lambda, x_ref, y_ref);

    vector<DubinsTrajectoryKind> options = { LSL, RSR, LSR, RSL, RLR, LRL };
    vector<DubinsTrajectory> trajectories;

    for (auto option: options) {
        auto t = solve(option, transformed_start.orientation, transformed_end.orientation, new_k_max);
        if (isnan(t.total_length)) {
            continue;
        }
        t.scale(lambda);
        trajectories.push_back(t);
    }

    // sort by shortest
    sort(trajectories.begin(), trajectories.end(), [](auto a, auto b) {
        return a.total_length < b.total_length;
    });

    for (auto t: trajectories) {
        auto final_trajectory = ExecutableDubinsTrajectory(t, start, kmax, velocity);
        auto points_to_check = final_trajectory.get_trajectory(collision_resolution);
        bool occupied = false;
        for (auto p: points_to_check) {
            if (!occupation.is_available(p.position)) {
                occupied = true;
                break;
            }
        }

        if (!occupied) {
            output = final_trajectory;
            return true;
        }
    }

    return false;
}

float mod2pi(float x) {
    float const two_pi = 2 * M_PI;
    while (x >= two_pi) x -= two_pi;
    while (x < 0) x += two_pi;
    return x;
}

DubinsTrajectory solve_lsl(float theta_start, float theta_end, float k) {
    float c = cos(theta_end) - cos(theta_start);
    float s = 2*k +  sin(theta_start) - sin(theta_end);
    float s1 = 1/k * mod2pi(atan2(c,s) - theta_start);
    float s2 = 1/k * sqrt(2 + 4*k*k - 2*cos(theta_start-theta_end) + 4*k*(sin(theta_start)-sin(theta_end)));
    float s3 = 1/k * mod2pi(theta_end - atan2(c,s));
    return {LSL, s1, s2, s3};
}


DubinsTrajectory solve(DubinsTrajectoryKind kind, float theta_start, float theta_end, float k) {
    switch (kind) {
        case LSL:
            return solve_lsl(theta_start, theta_end, k);
        case RSR:
            return solve_rsr(theta_start, theta_end, k);
        case LSR:
            return solve_lsr(theta_start, theta_end, k);
        case RSL:
            return solve_rsl(theta_start, theta_end, k);
        case RLR:
            return solve_rlr(theta_start, theta_end, k);
        case LRL:
            return solve_lrl(theta_start,theta_end,k);
        }
}

DubinsTrajectory solve_rsr(float theta_start, float theta_end, float k) {
    float c = cos(theta_start) - cos(theta_end);
    float s = 2 * k - sin(theta_start) + sin(theta_end);
    float s1 = (1.0f / k) * mod2pi(theta_start - atan2(c, s));
    float s2_inner = 2.0f + 4.0f * k * k
                     - 2.0f * cos(theta_start - theta_end)
                     - 4.0f * k * (sin(theta_start) - sin(theta_end));
    float s2 = (1.0f / k) * sqrt(s2_inner);
    float s3 = (1.0f / k) * mod2pi(atan2(c, s) - theta_end);
    return {RSR, s1, s2, s3};
}

DubinsTrajectory solve_lsr(float theta_start, float theta_end, float k) {
    float c = cos(theta_start) + cos(theta_end);
    float s = 2 * k + sin(theta_start) + sin(theta_end);

    float s2_inner = -2.0f + 4.0f * k * k
                     + 2.0f * cos(theta_start - theta_end)
                     + 4.0f * k * (sin(theta_start) + sin(theta_end));
    float s2 = (1.0f / k) * sqrt(s2_inner);

    float common_atan = atan2(-c, s) - atan2(-2.0f, k * s2);

    float s1 = (1.0f / k) * mod2pi(common_atan - theta_start);
    float s3 = (1.0f / k) * mod2pi(common_atan - theta_end);

    return {LSR, s1, s2, s3};
}

DubinsTrajectory solve_rsl(float theta_start, float theta_end, float k) {
    float c = cos(theta_start) + cos(theta_end);
    float s = 2 * k - sin(theta_start) - sin(theta_end);

    float s2_inner = -2.0f + 4.0f * k * k + 2.0f * cos(theta_start - theta_end) - 4.0f * k * (sin(theta_start) + sin(theta_end));
    float s2 = (1.0f / k) * sqrt(s2_inner);

    float common_atan = atan2(c, s) - atan2(2.0f, k * s2);
    float s1 = (1.0f / k) * mod2pi(theta_start - common_atan);
    float s3 = (1.0f / k) * mod2pi(theta_end - common_atan);

    return {RSL, s1, s2, s3};
}

DubinsTrajectory solve_rlr(float theta_start, float theta_end, float k) {
    float c = cos(theta_start) - cos(theta_end);
    float s = 2 * k - sin(theta_start) + sin(theta_end);

    float arg = 0.125f * (6.0f - 4.0f * k * k + 2.0f * cos(theta_start - theta_end) + 4.0f * k * (sin(theta_start) - sin(theta_end)));
    float s2 = (1.0f / k) * mod2pi(2.0f * M_PI - acos(arg));

    float s1 = (1.0f / k) * mod2pi(theta_start - atan2(c, s) + 0.5f * k * s2);
    float s3 = (1.0f / k) * mod2pi(theta_start - theta_end + k * (s2 - s1));

    return {RLR, s1, s2, s3};
}

DubinsTrajectory solve_lrl(float theta_start, float theta_end, float k) {
    float c = cos(theta_end) - cos(theta_start);
    float s = 2 * k + sin(theta_start) - sin(theta_end);

    float s2 = (1.0f / k) * mod2pi(2.0f * M_PI - acos(0.125f * (6.0f - 4.0f * k * k + 2.0f * cos(theta_start - theta_end) - 4.0f * k * (sin(theta_start) - sin(theta_end)))));

    float s1 = (1.0f / k) * mod2pi(-theta_start + atan2(c, s) + 0.5f * k * s2);
    float s3 = (1.0f / k) * mod2pi(theta_end - theta_start + k * (s2 - s1));

    return {LRL, s1, s2, s3};
}

Pose direct_transformation(Pose p, float phi, float lambda, float x_ref, float y_ref) {
    float x = 1/lambda * (p.position.x * cos(phi) + p.position.y * sin(phi) - x_ref);
    float y = 1/lambda * (p.position.x * -sin(phi) + p.position.y * cos(phi) - y_ref);
    float angle = p.orientation - phi;
    return {{x,y}, angle};
}

DubinsTrajectory::DubinsTrajectory(DubinsTrajectoryKind kind, float t1, float t2, float t3) {
    this->kind = kind;
    this->t1 = t1;
    this->t2 = t2;
    this->t3 = t3;
    this->total_length = t1 + t2 + t3;
}

DubinsTrajectory::DubinsTrajectory() {
    this->kind = LRL;
    this->t1 = 0;
    this->t2 = 0;
    this->t3 = 0;
    this->total_length = 0;
}

void DubinsTrajectory::scale(float lambda) {
    this ->t1 *= lambda;
    this ->t2 *= lambda;
    this ->t3 *= lambda;
    this->total_length = t1 + t2 + t3;
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



Pose Trajectory::operator()(float time) const {
    throw logic_error("not implemented");
}

Velocities Trajectory::get_velocities(float time) const {
    throw logic_error("not implemented");
}


Pose RotatingTrajectory:: operator()(float time) const{
    auto position = center + Point::FromPolar(
        phi + time * omega,
        radius
    );
    float orientation = phi + time * omega + (omega > 0? M_PI / 2 : -M_PI / 2);
    return {position, orientation};
}

Velocities RotatingTrajectory::get_velocities(float time) const {
    return {omega * radius, omega};
}

RotatingTrajectory::RotatingTrajectory(Point center, float radius, float omega, float phi) {
    this->center = center;
    this->radius = radius;
    this->omega = omega;
    this->phi = phi;
}
RotatingTrajectory RotatingTrajectory::LTrajectory(Pose initial_pose, float radius, float speed) {
    Point center = initial_pose.position + Point::FromPolar(initial_pose.orientation + M_PI/2, radius);
    float phi = atan2(initial_pose.position.y - center.y, initial_pose.position.x - center.x);
    float omega = speed / radius;
    return RotatingTrajectory(center, radius, omega, phi);
}
RotatingTrajectory RotatingTrajectory::RTrajectory(Pose initial_pose, float radius, float speed) {
    Point center = initial_pose.position + Point::FromPolar(initial_pose.orientation - M_PI/2, radius);
    float phi = atan2(initial_pose.position.y - center.y, initial_pose.position.x - center.x);
    float omega = - speed / radius;
    return RotatingTrajectory(center, radius, omega, phi);
}

Pose StraightTrajectory:: operator()(float time) const {
    auto position =  initial_pose.position + Point::FromPolar(initial_pose.orientation, speed * time);
    auto orientation = initial_pose.orientation;
    return {position, orientation};
}

Velocities StraightTrajectory::get_velocities(float time) const {
    return {speed, 0};
}

StraightTrajectory::StraightTrajectory(Pose initial_pose, float speed) {
    this->initial_pose = initial_pose;
    this->speed = speed;
}

variant<RotatingTrajectory, StraightTrajectory> build_trajectory(SingleTrajectoryKind trajectory_kind, float k, float v, Pose start) {
    float radius = 1/k;
    switch (trajectory_kind) {
        case L:
            return RotatingTrajectory::LTrajectory(start, radius, v);
        case R:
            return RotatingTrajectory::RTrajectory(start, radius, v);
        case S:
            return StraightTrajectory(start, v);
        default:
            assert(false);
    }
}


Pose execute(variant<RotatingTrajectory, StraightTrajectory> const& trajectory, float time) {
    return std::visit([&](auto& arg) { return arg(time); }, trajectory);
}

Pose ExecutableDubinsTrajectory::operator()(float time) {
    if (time <= this->t1) {
        return execute(this->trajectory_1, time);
    }
    if (time <= this->t2) {
        return execute(this->trajectory_2, time - this->t1);
    }
    return execute(this->trajectory_3, time - this->t2);
}

ExecutableDubinsTrajectory::ExecutableDubinsTrajectory(DubinsTrajectory trajectory, Pose start, float k, float v) {
    auto kind_t1 = slice_trajectory(trajectory.kind, 0);
    auto kind_t2 = slice_trajectory(trajectory.kind, 1);
    auto kind_t3 = slice_trajectory(trajectory.kind, 2);

    this->t1 = trajectory.t1 / v;
    this->t2 = this->t1 + trajectory.t2 / v;

    // fist trajectory
    this->trajectory_1 = build_trajectory(kind_t1, k, v, start);
    Pose end_1 = execute(this->trajectory_1, this->t1);

    // second trajectory
    this->trajectory_2 = build_trajectory(kind_t2, k, v, end_1);
    Pose end_2 = execute(this->trajectory_2, this->t2-this->t1);

    // third trajectory
    this->trajectory_3 = build_trajectory(kind_t3, k, v, end_2);

    this->length = trajectory.total_length;
    this->time = length/v;
}


ExecutableDubinsTrajectory::ExecutableDubinsTrajectory() {
    this->length = 0;
    this->time = 0;
    this->t1 = 0;
    this->t2 = 0;
}

vector<Pose> ExecutableDubinsTrajectory::get_trajectory(int resolution) {
    int points = length*resolution;
    vector<Pose> trajectory = {};
    trajectory.reserve(points+1);

    for (int i = 0; i <= points; i++) {
        float t = static_cast<float>(i) / static_cast<float>(points) * time;
        trajectory.push_back(this->operator()(t));
    }

    return trajectory;
}

Velocities execute_v(variant<RotatingTrajectory, StraightTrajectory> const& trajectory, float time) {
    return std::visit([&](auto& arg) { return arg.get_velocities(time); }, trajectory);
}


Velocities ExecutableDubinsTrajectory::get_velocities(float time) const {
    if (time <= this->t1) {
        return execute_v(this->trajectory_1, time);
    }
    if (time <= this->t2) {
        return execute_v(this->trajectory_2, time - this->t1);
    }
    return execute_v(this->trajectory_3, time - this->t2);
}


void ExecutableDubinsTrajectory::debug(int resolution) {
    auto trajectory = get_trajectory(resolution);
    vector<Point> points;
    for (auto p : trajectory) {
        points.push_back(p.position);
    }
    display({}, points);
}

Velocities::Velocities(float velocity, float angular_velocity) {
    this->velocity = velocity;
    this->angular_velocity = angular_velocity;
}
