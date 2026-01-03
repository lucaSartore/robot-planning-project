#include "combinatorial_graph_builder.hpp"
#include "../triangulation/triangulation.hpp"

Graph CombinatorialGraphBuilder::convert(Map map) {
    vector<Point> points = {};
    // insert all the map borders
    points.insert(points.end(), map.borders.begin(), map.borders.end());

    // insert all obstacles
    for (auto obstacle: map.obstacles) {
        if (obstacle.kind == ObstacleKind::Cylinder) {
            // approximate using rectangle
            Point center = obstacle.cylinder.center;
            float radius = obstacle.cylinder.radius;

            points.push_back(center + Point(radius, radius));
            points.push_back(center + Point(radius, -radius));
            points.push_back(center + Point(-radius, radius));
            points.push_back(center + Point(-radius, -radius));
        } else {
            auto to_insert = obstacle.polygon.points;
            points.insert(points.end(), to_insert.begin(), to_insert.end());
        }
    }

    auto triangles = triangulate(points);

    return Graph(points);

}
