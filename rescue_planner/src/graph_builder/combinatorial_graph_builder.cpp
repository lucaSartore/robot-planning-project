#include "combinatorial_graph_builder.hpp"
#include "../triangulation/triangulation.hpp"
#include <assert.h>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include "../util/display.hpp"

void group_points(Map const& map, vector<Point> &grouped_points, vector<tuple<int,int>> & obstacles_vertexes, vector<int> & points_label);
Graph build_graph(Map const& map, vector<Triangle> const& triangles, vector<int> const& points_label, vector<Point> const& points);


Graph CombinatorialGraphBuilder::convert(Map map) {
    vector<Point> grouped_points = {};
    vector<int> points_labels {};
    vector<tuple<int,int>> obstacle_vertexes = {};
    group_points(map, grouped_points, obstacle_vertexes, points_labels);
    auto triangles = triangulate(grouped_points, obstacle_vertexes);

    auto graph = build_graph(map, triangles, points_labels, grouped_points);


    auto debug_data = graph.get_debug_data();
    auto lines = std::get<0>(debug_data);
    // lines = {};
    // auto points = std::get<1>(debug_data);
    // for (auto a : obstacle_vertexes) {
    //     lines.push_back({
    //         grouped_points[std::get<0>(a)],
    //         grouped_points[std::get<1>(a)]
    //     });
    // }
    // display(lines, points);

    return graph;
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

    // adding the exit (connected only to the center as it is already a side)
    int exit_triangle_index = find_triangle_that_include_point(triangles, points, map.exit.position);
    assert(exit_triangle_index != -1);
    auto exit_triangle = triangles[exit_triangle_index];
    auto exit_triangle_center = (points[exit_triangle.a] + points[exit_triangle.b] + points[exit_triangle.c]) / 3.0;
    auto exit_triangle_point = point_to_index[exit_triangle_center];
    graph.add_adjacent(exit_triangle_point, exit_node);

    auto add_connections_to_triangle = [&](int node_index, int triangle_index) {
        auto triangle = triangles[triangle_index];
        auto p1 = points[triangle.a];
        auto p2 = points[triangle.b];
        auto p3 = points[triangle.c];

        if (points_label[triangle.a] != points_label[triangle.b]) {
            int i1 = point_to_index[(p1+p2)/2];
            graph.add_adjacent(node_index, i1);
        }
        if (points_label[triangle.b] != points_label[triangle.c]) {
            int i2 = point_to_index[(p2+p3)/2];
            graph.add_adjacent(node_index, i2);
        }
        if (points_label[triangle.c] != points_label[triangle.a]) {
            int i3 = point_to_index[(p3+p1)/2];
            graph.add_adjacent(node_index, i3);
        }
    };

    // adding the robot position
    int robot_triangle_index = find_triangle_that_include_point(triangles, points, map.exit.position);
    assert(robot_triangle_index != -1);
    add_connections_to_triangle(robot_position, robot_triangle_index);

    // adding points to the victims
    for (int i=0; i<map.victims.size(); i++) {
        auto victim = map.victims[i];
        int victim_triangle_index = find_triangle_that_include_point(triangles, points, victim.position);
        // victim_triangle_index = 0;
        assert(victim_triangle_index != -1);
        add_connections_to_triangle(i, victim_triangle_index);
    }


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


    for (int i=0; i<all_graph_points.size(); i++) {
        if (graph.nodes[i].adjacent.size() == 0) {
            graph.nodes.erase(i);
        }
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
            points_label.push_back(i);
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
