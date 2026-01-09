#include "sampling_graph_builder.hpp"
#include "../triangulation/triangulation.hpp"
#include <random>

Graph SamplingGraphBuilder::convert(Map map) {
    auto points = sample_points();
    int robot_position, exit_position;
    vector<int> victims_position;

    robot_position = points.size();
    points.push_back(map.robot_position.position);

    exit_position = points.size();
    points.push_back(map.exit.position);

    for (auto& v: map.victims) {
        victims_position.push_back(points.size());
        points.push_back(v.position);
    }

    Graph graph = Graph(
        points,
        robot_position,
        exit_position,
        victims_position
    );

    Kdtree::KdNodeVector kd_nodes = {};
    for (int i = 0; i < points.size(); i++) {
        kd_nodes.push_back({{points[i].x, points[i].y}, nullptr, i});
    }

    auto kd_tree = Kdtree::KdTree(&kd_nodes);
    for (int i = 0; i < points.size(); i++) {
        auto kd_node = kd_nodes[i];
        Kdtree::KdNodeVector nearest = {};
        kd_tree.k_nearest_neighbors(kd_node.point, k_nearest, &nearest);

        for (int j=0; j<nearest.size(); j++) {
            int second_index = nearest[j].index;
            if (i == second_index) {
                continue;
            }
            if (verify_connectivity(points[i], points[second_index])) {
                graph.add_adjacent(i,second_index);
            }
        }
    }

    return graph;
}

bool SamplingGraphBuilder::verify_connectivity(Point start, Point end, int resolution) {
    float d = distance(start, end);
    int p = d * resolution;
    Point v = end - start;
    for (int i=0; i<=p; i++) {
        auto result = start + v * ((float)i/float(p));
        if (!occupation.is_available(result)) {
            return false;
        }
    }
    return true;
}

vector<Point> SamplingGraphBuilder::sample_points() {
   vector<Point> points;
    points.reserve(points.size());

    while (points.size() < number_of_points) {
        auto x = rand()/(float)RAND_MAX;
        auto y = rand()/(float)RAND_MAX;
        x = x * (occupation.x_max - occupation.x_min) + occupation.x_min;
        y = y * (occupation.y_max - occupation.y_min) + occupation.y_min;
        if (occupation.is_available({x,y})) {
            points.push_back({x,y});
        }
    }

    return points;
}

