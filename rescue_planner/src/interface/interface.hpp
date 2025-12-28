#pragma once

#include <vector>

using namespace std;

class Point {
public:
    float x;
    float y;
};

class Exit {
public:
    Point position;
    // orientation of the exit (in degrees,
    // 0 is facing upword, clockwise)
    float orientation;
};

class PolygonObstacle {
public:
    vector<Point> points;
};


class CylinderObstacle {
public:
    Point center;
    float radius;
};

enum ObstacleKind {
    Polygon,
    Cylinder
};

class Obstacle {
public:
    ObstacleKind kind;

    union {
        PolygonObstacle polygon;
        CylinderObstacle cylinder;
    } value;
};


class Victim {
public:
    Point position;
    float value;
};

class Map {
public:
    Exit exit;
    vector<Point> borders;
    vector<Victim> victims;
    vector<Obstacle> obstacles;
};



class Interface {
public:
    virtual Map GetMap();
    virtual void OutputTrajectory(vector<Point> trajectory);
};
