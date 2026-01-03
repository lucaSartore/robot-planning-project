#include "triangulation.hpp"

#include <algorithm>
#include <assert.h>
#include <math.h>
#include <tuple>
#include <set>
#include <map>

tuple<Triangle, Point> find_first_triangle(std::vector<Point> points);
vector<int> sort_indexes_by_distance(std::vector<Point> points, Point center, Triangle triangle_to_exclude);
bool is_visible(std::vector<Point> points, int source, std::set<int> perimeter_points, std::tuple<int, int> side);

int triangulate(std::vector<Point> points, std::vector<Triangle>& triangles) {
    assert(points.size() > 3);
    auto first_triangle_tuple = find_first_triangle(points);
    auto first_triangle = std::get<0>(first_triangle_tuple);
    auto center = std::get<1>(first_triangle_tuple);

    // indexes sorted by distance (usefull for the next reconstruction)
    auto indexes_sorted_by_distance = sort_indexes_by_distance(points, center, first_triangle);
}


std::vector<Triangle> gradual_expansion(std::vector<Point> points, Point center, vector<int> indexes_sorted_by_distance, Triangle seed) {
    std::set<int> perimeter_points = { seed.a, seed.b, seed.c };
    std::map<int,int> point_to_clockwise_neighbour = {
        {seed.a, seed.b},
        {seed.b, seed.c},
        {seed.c, seed.a},
    };
    std::map<int,int> point_to_counter_clockwise_neighbour = {
        {seed.a, seed.c},
        {seed.c, seed.b},
        {seed.b, seed.a},
    };
    vector<Triangle> triangles = {seed};

    for (auto p_index: indexes_sorted_by_distance) {
        p = points[p_index];

    }
}

/// given a convex shape made of perimeter_points point and a side of that perimeter
/// this function true if from a certain point of view (source) the side is visible
bool is_visible(std::vector<Point> points, int source, std::set<int> perimeter_points, std::tuple<int, int> side) {

    auto side_1_index = std::get<0>(side);
    auto side_2_index = std::get<1>(side);
    auto side_1 = points[side_1_index];
    auto side_2 = points[side_2_index];

    // special case when the equation has form x > n instead of y > mx + q
    bool special_case = std::abs(side_1.x - side_2.x) < 1e-6;

    float n=0; m=0; q=0;

    if (special_case) {
        n = side_1.x;
    } else {
        m = (side_1.y - side_2.y) / (side_1.x - side_2.x);
        q = (side_1.x * side_2.y - side_2.x *side_1.y) / (side_1.x - side_2.x);
    }

    auto unequality = [&](int a) {
        auto point = points[a];
        if (special_case) {
            return point.x > n;
        } else {
            return point.y > m * point.x + q;
        }
    };

    bool side_of_plane_of_source = unequality(source);
    for (auto p: perimeter_points) {
        if (p == side_1_index || p == side_2_index) {
            continue;
        }
        bool side_of_plane_of_p = unequality(p);
        if (side_of_plane_of_p == side_of_plane_of_source) {
            return false;
        }
    }

    return true;
}

vector<int> sort_indexes_by_distance(std::vector<Point> points, Point center, Triangle triangle_to_exclude) {
    vector<int> indexes_sorted_by_distance = {};
    for (int i = 0; i < points.size(); i++) {
        if (
            i == triangle_to_exclude.a ||
            i == triangle_to_exclude.b ||
            i == triangle_to_exclude.c ) {
            continue;
            }
        indexes_sorted_by_distance.push_back(i);
    }
    auto cmp = [&](int a, int b) {
        float da = distance(center, points[a]);
        float db = distance(center, points[b]);
        return da < db;
    };
    std::sort(indexes_sorted_by_distance.begin(), indexes_sorted_by_distance.end(), cmp);
    return indexes_sorted_by_distance;
}

tuple<Point, float> find_circum_circle(Point p1, Point p2, Point p3) {
    // Calculate the determinant D
    float D = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));

    // If D is 0, the points are collinear and no circle exists
    if (std::abs(D) < 1e-6) {
        return { {0, 0}, -1.0f };
    }

    float p1_sq = p1.x * p1.x + p1.y * p1.y;
    float p2_sq = p2.x * p2.x + p2.y * p2.y;
    float p3_sq = p3.x * p3.x + p3.y * p3.y;

    float centerX = (p1_sq * (p2.y - p3.y) + p2_sq * (p3.y - p1.y) + p3_sq * (p1.y - p2.y)) / D;
    float centerY = (p1_sq * (p3.x - p2.x) + p2_sq * (p1.x - p3.x) + p3_sq * (p2.x - p1.x)) / D;

    Point center = {centerX, centerY};

    // Radius is the distance from center to any point (e.g., p1)
    float radius = std::sqrt(std::pow(center.x - p1.x, 2) + std::pow(center.y - p1.y, 2));

    return {center, radius};
}

float distance(Point p1, Point p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

tuple<Triangle, Point> find_first_triangle(std::vector<Point> points) {
    // pick p1 at random
    int p1 = 0;

    vector<float> tmp;

    // pick p2 so that it is the closest
    int p2 = 1;
    float d_p1_p2 = distance(points[p1], points[p2]);
    for (int i = 2; i < points.size(); i++) {
        float d_p1_i = distance(points[p1], points[i]);
        if ( d_p1_i < d_p1_p2 ) {
            d_p1_p2 = d_p1_i;
            p2 = i;
        }
    }

    // pick p3 so that the circle area is the smallest
    int p3 = 2;
    float radius_with_p3 = std::get<1>(
        find_circum_circle( points[p1], points[p2], points[p3] )
    );
    for (int i = 3; i < points.size(); i++) {

        if (i == p2) {
            continue;
        }

        float radius_with_i = std::get<1>(
            find_circum_circle( points[p1], points[p2], points[i] )
        );

        if ( radius_with_i < radius_with_p3 ) {
            radius_with_p3 = radius_with_i;
            p3 = i;
        }
    }
    Point center = std::get<0>(
        find_circum_circle( points[p1], points[p2], points[p3] )
    );

    return {{p1, p2, p3}, center};
}