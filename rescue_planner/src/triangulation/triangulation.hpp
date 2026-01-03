#pragma once
#include "../interface/interface.hpp"
#include <vector>

class Triangle{
public:
    /// indexes of the one of points of the 3 edges
    int a;
    /// indexes of the one of points of the 3 edges
    int b;
    /// indexes of the one of points of the 3 edges
    int c;
};


tuple<Point, float> find_circum_circle(Point p1, Point p2, Point p3);
float distance(Point p1, Point p2);
std::vector<Triangle> triangulate(std::vector<Point> points);


