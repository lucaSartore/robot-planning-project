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
    Triangle(int a, int b, int c);
    bool operator==(const Triangle& t) const;
};

template<> struct std::hash<Triangle> {
    std::size_t operator()(Triangle const& s) const noexcept {
        std::size_t h1 = std::hash<int>{}(s.a);
        std::size_t h2 = std::hash<int>{}(s.b);
        std::size_t h3 = std::hash<int>{}(s.c);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};


tuple<Point, float> find_circum_circle(Point p1, Point p2, Point p3);
float distance(Point p1, Point p2);

std::vector<Triangle> triangulate(std::vector<Point> const& points, vector<tuple<int,int>> const& obstacles_vertexes);

int find_triangle_that_include_point(vector<Triangle> const& triangles, vector<Point> const& points, Point point, float margin = 0);
