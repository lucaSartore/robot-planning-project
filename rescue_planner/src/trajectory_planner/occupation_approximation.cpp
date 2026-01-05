#include <map>

#include "trajectory_planner.hpp"
#include "../interface/interface.hpp"
#include "algorithm"
// OccupationApproximation is a class that memorize the points on a map that are "occupated"
// (i.e.) the robot can't be in it by discretizing the space and creating a boolean map

tuple<int,int> OccupationApproximation::get_indexes(float x, float y) const {
    // this convert the coordinates on the plane inside the
    // coordinates of the approximated boolean map

}

bool OccupationApproximation::get(float x, float y) const {
    auto indexes = get_indexes(x, y);
}
bool OccupationApproximation::get(Point point) const {
    return get(point.x, point.y);
}


void OccupationApproximation::set(float x, float y, bool vlaue) {
    auto indexes = get_indexes(x, y);
}
void OccupationApproximation::set(Point point, bool vlaue) {
    set(point.x, point.y, vlaue);
}

vector<vector<bool>> OccupationApproximation::get_kernel(int radius) {
    // return a circular kernel with radius `radius` in a matrix
    // sith size (2*radius+1)*(2*radius+1)
}



void OccupationApproximation::draw_line(Point start, Point end) {
    float x = start.x;
    float y = start.x;
}
void OccupationApproximation::draw_circle(Point center, float radius) {
}

void OccupationApproximation::draw_map(Map const& map) {

    // draw the line and circles in the map

    // note: polycons and circle can only be a line and not be full in the beginning
    // this is because we will dilate them later


    //this are the interesting iteresting part of a map
    vector<Point> borders = map.borders;
    vector<Obstacle> obstacles = map.obstacles;

    // here is an example on how to ouse obstacles:
    for (auto o: obstacles) {
        if (o.kind == Polygon) {
            vector<Point> polygon_borders = o.polygon.points;
        } else if (o.kind == Cylinder) {
            Point center = o.cylinder.center;
            float radius = o.cylinder.radius;
        }
    }
}

vector<vector<bool>> OccupationApproximation::dilation(vector<vector<bool>> const& source, vector<vector<bool>> const& kernel) {

}

OccupationApproximation::OccupationApproximation(Map const &map, int resolution_x, float radius, float margins) {

    vector<float> x_values = {};
    vector<float> y_values = {};
    for (auto p: map.borders) {
        x_values.push_back(p.x);
        y_values.push_back(p.x);
    }

    this->x_max = *std::max_element(x_values.begin(), x_values.end()) + margins;
    this->y_max = *std::max_element(y_values.begin(), y_values.end()) + margins;
    this->x_min = *std::min_element(x_values.begin(), x_values.end()) - margins;
    this->y_min = *std::min_element(y_values.begin(), y_values.end()) - margins;

    int resolution_y = resolution_x / (x_max-x_min) * (y_max-y_min);

    this->resolution_x = resolution_x;
    this->resolution_y = resolution_y;


    this->ocupations = {};
    this->ocupations.reserve(resolution_x);
    for (int i=0; i<resolution_x; i++) {
        vector<bool> x = {};
        x.reserve(resolution_y);
        for (int j=0; j<resolution_y; j++) {
            x.push_back(false);
        }
        this->ocupations.push_back(x);
    }

    // putting the elements in the map
    draw_map(map);

    // generate kernel for dilation
    int radius_in_pixels = radius / (x_max-x_min) * this->resolution_x;
    auto kernel = get_kernel(radius);

    this->ocupations = dilation(this->ocupations, kernel);
}

bool OccupationApproximation::is_available(Point point) const {
    return !this->get(point);
}

