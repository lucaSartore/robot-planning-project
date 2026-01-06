#include "debug_interface.hpp"
#include "interface.hpp"
#include <iostream>


Map DebugInterface::GetMap(){
	// Build and return a fixed map for debugging/testing
	Pose exit = Pose(Point(-7.54319f, 3.05532f), 2.61799f);
	Pose robot = Pose(Point(0.0199976f, 0.000290901f), 0.00250393f);

	std::vector<Point> borders = {
		Point(-5.0f, 8.66025f), Point(5.0f, 8.66025f), Point(10.0f, 0.0f),
		Point(5.0f, -8.66025f), Point(-5.0f, -8.66025f), Point(-10.0f, 0.0f)
	};

	std::vector<Victim> victims = {
		Victim(Point(3.92794f, 2.98395f), 404.0f),
		Victim(Point(-4.67294f, 3.74056f), 199.0f),
		Victim(Point(-3.70581f, 6.97319f), 176.0f),
		Victim(Point(2.76273f, -2.71162f), 231.0f)
	};

	std::vector<Obstacle> obstacles;
	obstacles.push_back(Obstacle::CreatePolygon(std::vector<Point>{
		Point(-4.2508f, -3.86247f), Point(-3.40492f, -3.56916f),
		Point(-3.6705f, -2.80327f), Point(-4.51637f, -3.09658f)
	}));

	obstacles.push_back(Obstacle::CreatePolygon(std::vector<Point>{
		Point(3.77019f, -6.73954f), Point(3.41289f, -7.1199f),
		Point(4.05361f, -7.72177f), Point(4.41092f, -7.34141f)
	}));

	obstacles.push_back(Obstacle::CreatePolygon(std::vector<Point>{
		Point(-1.60594f, -1.94086f), Point(-1.22636f, -2.8486f),
		Point(-0.473252f, -2.53369f), Point(-0.852827f, -1.62595f)
	}));

	Map m = Map(exit, robot, borders, victims, obstacles);
	return m;
}

void DebugInterface::OutputTrajectory(vector<Pose> trajectory) {
	std::cout << "Trajectory output (debug):" << std::endl;
	for (const auto& p : trajectory) std::cout << p << std::endl;
}
