#include "trajectory_planner.hpp"
#include "math.h"

#include <iostream>

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

DubinsTrajectoryRaw::DubinsTrajectoryRaw(vector<Point> trajectory) {
    this->trajectory = trajectory;
}

DubinsTrajectoryRaw::DubinsTrajectoryRaw(DubinsTrajectory trajectory, Pose start, float k, float v, int resolution) {
    this->trajectory = {};
}

