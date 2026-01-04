#include "combinatorial_graph_builder.hpp"
#include "../triangulation/triangulation.hpp"
#include <assert.h>
#include <tuple>
#include <unordered_set>
#include <unordered_map>

void group_points(Map const& map, vector<Point> &grouped_points, vector<tuple<int,int>> & obstacles_vertexes, vector<int> & points_label);
Graph build_graph(Map const& map, vector<Triangle> const& triangles, vector<int> const& points_label, vector<Point> const& points);


Graph CombinatorialGraphBuilder::convert(Map map) {
    vector<Point> grouped_points = {};
    vector<int> points_labels {};
    vector<tuple<int,int>> obstacle_vertexes = {};
    group_points(map, grouped_points, obstacle_vertexes, points_labels);
    auto triangles = triangulate(grouped_points, obstacle_vertexes);

    return build_graph(map, triangles, points_labels, grouped_points);
}

Graph build_graph(Map const& map, vector<Triangle> const& triangles, vector<int> const& points_label, vector<Point> const& points) {
    unordered_set<Point> all_graph_points_set = {};
    for (auto triangle: triangles) {
        auto p1 = points[triangle.a];
        auto p2 = points[triangle.b];
        auto p3 = points[triangle.c];

        auto c = (p1 + p2 + p3) / 3.0;
        auto v1 = (p1 + p2) / 2.0;
        auto v2 = (p2 + p3) / 2.0;
        auto v3 = (p1 + p3) / 2.0;

        all_graph_points_set.insert(c);
        all_graph_points_set.insert(v1);
        all_graph_points_set.insert(v2);
        all_graph_points_set.insert(v3);
    }
    vector<Point> all_graph_points = {};
    all_graph_points.reserve(all_graph_points_set.size() + map.victims.size() + 2);

    unordered_map<Point, int> point_to_index = {};
    for (auto p: all_graph_points_set) {
        all_graph_points.push_back(p);
        point_to_index[p] = all_graph_points.size() - 1;
    }

    int exit_node;
    int robot_position;
    vector<int> victims_position = {};


    for (auto v: map.victims) {
        all_graph_points.push_back(v.position);
        victims_position.push_back(all_graph_points.size() - 1);
    }

    all_graph_points.push_back(map.exit.position);
    exit_node = all_graph_points.size() - 1;

    all_graph_points.push_back(map.robot_position.position);
    robot_position = all_graph_points.size() - 1;



    Graph  graph = {all_graph_points, exit_node, robot_position, victims_position};

    auto insert_vertex = [&](Point c, Point v1, Point v2, int iv1, int iv2) {
        // avoid adding a point that goes to a wall
        if (points_label[iv1] == points_label[iv2]) {
            return;
        }
        auto p_exit = (v1 + v2) / 2.0;

        int pc_index = point_to_index[c];
        int p_exit_index = point_to_index[p_exit];

        graph.add_adjacent(pc_index, p_exit_index);
    };
    for (auto triangle: triangles) {
        auto ip1 = triangle.a;
        auto ip2 = triangle.b;
        auto ip3 = triangle.c;
        auto p1 = points[ip1];
        auto p2 = points[ip2];
        auto p3 = points[ip3];

        auto c = (p1 + p2 + p3) / 3.0;

        insert_vertex(c, p1, p2, ip1, ip2);
        insert_vertex(c, p2, p3, ip2, ip3);
        insert_vertex(c, p3, p1, ip3, ip1);
    }

    return graph;
}


void group_points(Map const& map, vector<Point> &grouped_points, vector<tuple<int,int>> & obstacles_vertexes, vector<int> & points_label) {
    vector<vector<Point>> points = {};
    // insert all the map borders
    points.push_back({});
    points[0].insert(points[0].end(), map.borders.begin(), map.borders.end());

    // insert all obstacles
    for (auto obstacle: map.obstacles) {
        points.push_back({});
        if (obstacle.kind == ObstacleKind::Cylinder) {
            // approximate using rectangle
            Point center = obstacle.cylinder.center;
            float radius = obstacle.cylinder.radius;

            points.back().push_back(center + Point(radius, radius));
            points.back().push_back(center + Point(radius, -radius));
            points.back().push_back(center + Point(-radius, radius));
            points.back().push_back(center + Point(-radius, -radius));
        } else {
            auto to_insert = obstacle.polygon.points;
            points.back().insert(points.back().end(), to_insert.begin(), to_insert.end());
        }
    }

    for (int i = 0; i < points.size(); i++) {
        // all obstacle must be polycons
        assert(points[i].size() >= 3);
        for (int j = 0; j < points[i].size(); j++) {
            points_label.push_back(j);
            grouped_points.push_back(points[i][j]);
            if (j != 0) {
                obstacles_vertexes.push_back({
                    grouped_points.size() - 1,
                    grouped_points.size() - 2
                });
            }
        }
        obstacles_vertexes.push_back({
            grouped_points.size() - 1,
            grouped_points.size() - points[i].size()
        });
    }
    assert(grouped_points.size() >= 3);
}
