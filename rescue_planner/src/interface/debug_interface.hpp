#pragma once

#include "interface.hpp"
#include <vector>

class DebugInterface: public Interface {
public:
    DebugInterface();
    virtual Map GetMap();
    virtual void OutputTrajectory(vector<Point> trajectory);
};
