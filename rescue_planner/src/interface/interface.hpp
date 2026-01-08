#pragma once

#include <vector>
#include <ostream>
#include <optional>
#include <tuple>
#include "common_types.hpp"
#include "../dubins_graph/dubins_graph.hpp"

using namespace std;


class Interface {
public:
    virtual ~Interface() {};
    virtual Map GetMap() = 0;
    virtual void OutputTrajectory(Result result) = 0;
};
