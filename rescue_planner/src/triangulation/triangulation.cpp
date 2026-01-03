#include "triangulation.hpp"

#include <algorithm>
#include <assert.h>
#include <iostream>
#include <math.h>
#include <tuple>
#include <set>
#include <map>
#include "../util/display.hpp"
#include <random>

tuple<Triangle, Point> find_first_triangle(std::vector<Point> points);
vector<int> sort_indexes_by_distance(std::vector<Point> points, Point center, std::vector<int> to_exclude);
bool is_visible(std::vector<Point> points, int source, std::vector<int> perimeter_points, std::tuple<int, int> side);
std::vector<Triangle> gradual_expansion(std::vector<Point> points, Point center, vector<int> indexes_sorted_by_distance, Triangle seed);
void debug(std::vector<Triangle> triangles, vector<Point> points);
bool are_points_inside_triangle(Triangle triangle, vector<int> points_indexes, vector<Point> const & points);
bool intersect(tuple<int, int> s1, tuple<int, int> s2, vector<Point> const& points);
bool intersect(tuple<int, int> s1, vector<tuple<int, int>> segments, vector<Point> const& points);

class LinearUnequality {
public:
    LinearUnequality(Point a, Point b, Point side_true_point) {
        special_case = std::abs(a.x - b.x) < 1e-6;

        n=0;
        m=0;
        q=0;

        if (special_case) {
            n = a.x;
        } else {
            m = (a.y - b.y) / (a.x - b.x);
            q = (a.x * b.y - b.x *a.y) / (a.x - b.x);
        }

        side_true = true;
        side_true = (*this)(side_true_point);
    }

    bool operator ()(Point p) {
        bool result;
        if (special_case) {
            result = p.x > n;
        } else {
            result =  p.y > m * p.x + q;
        }
        return result == side_true;
    }

    void flip() {
        side_true = !side_true;
    }

private:
    bool side_true;
    bool special_case;
    float n;
    float m;
    float q;
};

std::vector<Triangle> triangulate(std::vector<std::vector<Point>> points) {
    std::vector<Triangle> triangles = {};
    std::vector<Point> merged_points = {};
    vector<tuple<int,int>> obstacles_vertexes = {};
    for (int i = 0; i < points.size(); i++) {
        // all obstacle must be polycons
        assert(points[i].size() >= 3);
        for (int j = 0; j < points[i].size(); j++) {
            merged_points.push_back(points[i][j]);
            if (j != 0) {
                obstacles_vertexes.push_back({
                    merged_points.size() - 1,
                    merged_points.size() - 2
                });
            }
        }
        obstacles_vertexes.push_back({
            merged_points.size() - 1,
            merged_points.size() - points[i].size()
        });
    }
    assert(merged_points.size() >= 3);


    vector<tuple<int, int>> valid_arches = {};

    for (int i=0; i<merged_points.size(); i++) {
        for (int j=i+1; j<merged_points.size(); j++) {
            if (!intersect({i,j}, obstacles_vertexes, merged_points)) {
                valid_arches.push_back({i,j});
            }
        }
    }

    vector<tuple<int, int>> selected_arches = {};

    auto rng = std::default_random_engine {};
    std::shuffle(valid_arches.begin(), valid_arches.end(), rng);
    for (auto valid_arch: valid_arches) {
        if (!intersect(valid_arch, selected_arches, merged_points)) {
            selected_arches.push_back(valid_arch);
        }
    }

    vector<tuple<Point, Point>> arches = {};
    for (auto a : selected_arches) {
        arches.push_back({
            merged_points[std::get<0>(a)],
            merged_points[std::get<1>(a)]
        });
    }
    display(arches,{});

    return triangles;
}

bool intersect(tuple<int, int> s1, vector<tuple<int, int>> segments, vector<Point> const& points) {
    for (auto s2: segments) {
        if (intersect(s1, s2, points)) {
            return true;
        }
    }
    return false;
}
bool intersect(tuple<int, int> s1, tuple<int, int> s2, vector<Point> const& points) {

    auto p1 = points[std::get<0>(s1)];
    auto p2 = points[std::get<1>(s1)];
    auto p3 = points[std::get<0>(s2)];
    auto p4 = points[std::get<1>(s2)];

    if (p1 == p3 || p2 == p3 || p1 == p4 || p2 == p4) {
        return false;
    }


    float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

    // If denominator is 0, lines are parallel (no intersection)
    if (den == 0) {
        return false;
    }

    float t1 = (p1.x * p2.y - p1.y * p2.x);
    float t2 = (p3.x * p4.y - p3.y * p4.x);

    float intersect_x = (t1 * (p3.x - p4.x) - (p1.x - p2.x) * t2) / den;
    float intersect_y = (t1 * (p3.y - p4.y) - (p1.y - p2.y) * t2) / den;

    Point intersection = {intersect_x, intersect_y};

    float d12 = distance(p1, p2);
    float d34 = distance(p3, p4);

    return distance(p1,intersection) < d12 && distance(p2,intersection) < d12 &&
        distance(p3,intersection) < d34 && distance(p4,intersection) < d34;
}

/// given a convex shape made of perimeter_points point and a side of that perimeter
/// this function true if from a certain point of view (source) the side is visible
bool are_points_inside_triangle(Triangle triangle, vector<int> points_indexes, vector<Point> const & points) {
    int p1_index = triangle.a;
    int p2_index = triangle.b;
    int p3_index = triangle.c;

    auto p1 = points[p1_index];
    auto p2 = points[p2_index];
    auto p3 = points[p3_index];

    auto c1 = LinearUnequality(p1, p2, p3);
    auto c2 = LinearUnequality(p2, p3, p1);
    auto c3 = LinearUnequality(p3, p1, p2);

    for (int p_index: points_indexes) {
        auto p = points[p_index];
        if (
            c1(p) &&
            c2(p) &&
            c3(p)
        ) {
            return true;
        }

    }
    return false;
}

vector<int> sort_indexes_by_distance(std::vector<Point> points, Point center, vector<int> to_exclude) {
    vector<int> indexes_sorted_by_distance = {};
    for (int i = 0; i < points.size(); i++) {
        if ( std::find(to_exclude.begin(), to_exclude.end(), i) != to_exclude.end() ) {
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

void debug(std::vector<Triangle> triangles, vector<Point> points) {
    vector<tuple<Point, Point>> display_lines = {};
    vector<Point> display_points = {};
    for (auto t: triangles) {
        auto a = points[t.a];
        auto b = points[t.b];
        auto c = points[t.c];
        display_points.push_back(a);
        display_points.push_back(b);
        display_points.push_back(c);
        display_lines.push_back({a,b});
        display_lines.push_back({b,c});
        display_lines.push_back({c,a});
    }
    display(display_lines, display_points);
}
