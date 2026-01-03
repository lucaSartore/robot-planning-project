#pragma once

#include <vector>
#include <ostream>
#include <optional>

using namespace std;

class Point {
public:
    float x;
    float y;

    Point(float x = 0.0f, float y = 0.0f);
    friend ostream& operator<<(ostream& os, const Point& p);
    bool operator ==(const Point point) const;
    void operator +=(const Point point);
    Point operator +(const Point point) const;
};

class Pose {
public:
    Point position;
    // orientation in radiants on the xy plane
    // zero is pointing to the right
    float orientation; 

    Pose(Point pos = Point(), float orient = 0.0f);
    friend ostream& operator<<(ostream& os, const Pose& e);
};

class PolygonObstacle {
public:
    vector<Point> points;
    PolygonObstacle() = default;
    PolygonObstacle(const vector<Point>& pts);
};

class CylinderObstacle {
public:
    Point center;
    float radius;
    CylinderObstacle(Point c = Point(), float r = 0.0f);
};

enum ObstacleKind { Polygon, Cylinder };

class Obstacle {
public:
    ObstacleKind kind;
    PolygonObstacle polygon;
    CylinderObstacle cylinder;

    // Static factory methods
    static Obstacle CreatePolygon(const vector<Point>& pts);
    static Obstacle CreateCylinder(Point center, float radius);

    friend ostream& operator<<(ostream& os, const Obstacle& o);

    Obstacle() {}
};

class Victim {
public:
    Point position;
    float value;

    Victim(Point pos = Point(), float val = 0.0f);
    friend ostream& operator<<(ostream& os, const Victim& v);
};

class Map {
public:
    Pose exit;
    Pose robot_position;
    vector<Point> borders;
    vector<Victim> victims;
    vector<Obstacle> obstacles;

    Map() = default;
    Map(Pose e, Pose r, vector<Point> b, vector<Victim> v, vector<Obstacle> o);
    friend ostream& operator<<(ostream& os, const Map& m);
};

class Interface {
public:
    virtual ~Interface() {}
    virtual Map GetMap() = 0;
    virtual void OutputTrajectory(vector<Point> trajectory) = 0;
};


