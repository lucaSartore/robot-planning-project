#include "combinatorial_graph_builder.hpp"
#include "../triangulation/triangulation.hpp"

Graph CombinatorialGraphBuilder::convert(Map map) {
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

    auto triangles = triangulate(points);

    return Graph({});

}
