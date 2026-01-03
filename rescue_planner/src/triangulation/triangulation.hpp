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

int triangulate(std::vector<Point> points, std::vector<Triangle> triangles);
