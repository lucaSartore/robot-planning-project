#include "triangulation.hpp"

#include <algorithm>
#include <iostream>
#include <tuple>
#include "../util/display.hpp"
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>

vector<int> sort_indexes_by_distance(std::vector<Point> points, Point center, std::vector<int> to_exclude);
void debug(std::vector<Triangle> triangles, vector<Point> points);
bool are_points_inside_triangle(Triangle triangle, vector<int> points_indexes, vector<Point> const & points);
bool intersect(tuple<int, int> s1, tuple<int, int> s2, vector<Point> const& points);
bool intersect(tuple<int, int> s1, vector<tuple<int, int>> segments, vector<Point> const& points);
vector<Triangle> build_triangles_from_arches(vector<tuple<int,int>> const& arches);

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

std::vector<Triangle> triangulate(std::vector<Point> const& points, vector<tuple<int,int>> const& obstacles_vertexes) {
    vector<tuple<int, int>> valid_arches = {};

    for (int i=0; i<points.size(); i++) {
        for (int j=i+1; j<points.size(); j++) {
            if (!intersect({i,j}, obstacles_vertexes, points)) {
                valid_arches.push_back({i,j});
            }
        }
    }

    vector<tuple<int, int>> selected_arches = {};

    auto rng = std::default_random_engine {};
    std::shuffle(valid_arches.begin(), valid_arches.end(), rng);
    for (auto valid_arch: valid_arches) {
        if (!intersect(valid_arch, selected_arches, points)) {
            selected_arches.push_back(valid_arch);
        }
    }

    // vector<tuple<Point, Point>> arches = {};
    // for (auto a : selected_arches) {
    //     arches.push_back({
    //         points[std::get<0>(a)],
    //         points[std::get<1>(a)]
    //     });
    // }
    // display(arches,{});

    return build_triangles_from_arches(selected_arches);
}

vector<Triangle> build_triangles_from_arches(vector<tuple<int,int>> const& arches) {
    unordered_set<Triangle> triangles = {};
    unordered_map<int, unordered_set<int>> neigbours = {};
    auto insert = [&](int start, int end) {
        if (neigbours.find(start) == neigbours.end()) {
            neigbours[start] = {};
        }
        neigbours[start].insert(end);
    };
    for (auto a : arches) {
        auto n1 = std::get<0>(a);
        auto n2 = std::get<1>(a);
        insert(n1, n2);
        insert(n2, n1);
    }

    for (auto a : arches) {

        auto n1 = std::get<0>(a);
        auto n2 = std::get<1>(a);

        auto s1 = neigbours[n1];
        auto s2 = neigbours[n2];

        auto v1 = vector(s1.begin(), s1.end());
        auto v2 = vector(s2.begin(), s2.end());
        std::sort(v1.begin(), v1.end());
        std::sort(v2.begin(), v2.end());

        vector<int> output = {};
        std::set_intersection( v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(output));

        for (auto third: output) {
            triangles.insert({n1, n2, third});
        }
    }
    return {triangles.begin(), triangles.end()};
};

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

float distance(Point p1, Point p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
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


Triangle::Triangle(int a, int b, int c) {
    // points are sorted to make sure
    // triangles are unique
    vector p = {a,b,c};
    std::sort(p.begin(), p.end());
    this->a = p[0];
    this->b = p[1];
    this->c = p[2];
}
bool Triangle::operator==(const Triangle& t) const {
    return a == t.a && b == t.b && c == t.c;
}
