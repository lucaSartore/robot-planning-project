#pragma once

#include "interface.hpp"
#include <vector>

class RosInterface: public Interface {
public:
    RosInterface();
    virtual Map GetMap();
    virtual void OutputTrajectory(vector<Point> trajectory);
};
