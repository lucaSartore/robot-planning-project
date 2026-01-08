#pragma once

#include "interface.hpp"
#include <vector>

class DebugInterface: public Interface {
public:
    virtual Map GetMap();
    virtual void OutputTrajectory(Result result);

};
