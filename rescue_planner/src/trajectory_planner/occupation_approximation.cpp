#include "trajectory_planner.hpp"


OccupationApproximation::OccupationApproximation(Map const &map, int resolution) {
    this->resolution = resolution;
    this->x_max = 0;
    this->y_max = 0;
    this->x_min = 0;
    this->y_min = 0;
    this->ocupations = {};
}

bool OccupationApproximation::is_available(Point point) {
    return true;
}

