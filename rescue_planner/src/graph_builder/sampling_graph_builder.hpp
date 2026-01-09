#include <list>

#include "graph_builder.hpp"
#include "../dubins_graph/dubins_graph.hpp"
#include "../interface/common_types.hpp"
#include "../include/kdtree/kdtree.hpp"

class SamplingGraphBuilder : public GraphBuilder {
public:
    virtual Graph convert(Map map);
    SamplingGraphBuilder(OccupationApproximation& occupation, int number_of_points, int k_nearest): occupation(occupation) {
        this->number_of_points = number_of_points;
        this->k_nearest = k_nearest;
    }
private:
    OccupationApproximation& occupation;
    int number_of_points;
    int k_nearest;
    vector<Point> sample_points();
    bool verify_connectivity(Point start, Point end, int resolution = 5);
};