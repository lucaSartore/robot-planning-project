#include "interface.hpp"

void Interface::OutputTrajectory(Result result) {};

Map Interface::GetMap() {
    throw std::logic_error("GetMap(): not implemented");
};

