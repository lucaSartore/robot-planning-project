#include "interface.hpp"

Map Interface::GetMap() {};
void Interface::OutputTrajectory(vector<Point> trajectory) {};

// Point
Point::Point(float x, float y) : x(x), y(y) {}

// Exit
Exit::Exit(Point pos, float orient) : position(pos), orientation(orient) {}

// PolygonObstacle
PolygonObstacle::PolygonObstacle(const vector<Point>& pts) : points(pts) {}

// CylinderObstacle
CylinderObstacle::CylinderObstacle(Point c, float r) : center(c), radius(r) {}

// Obstacle Static Factories
Obstacle Obstacle::CreatePolygon(const vector<Point>& pts) {
    Obstacle o;
    o.kind = Polygon;
    o.value.polygon = PolygonObstacle(pts);
    return o;
}

Obstacle Obstacle::CreateCylinder(Point center, float radius) {
    Obstacle o;
    o.kind = Cylinder;
    o.value.cylinder = CylinderObstacle(center, radius);
    return o;
}

Obstacle::Obstacle(const Obstacle& other) {
    this->kind = other.kind;
    if (this->kind == Polygon) {
        this->value.polygon = other.value.polygon;
    } else {
        this->value.cylinder = other.value.cylinder;
    }
}

Obstacle& Obstacle::operator=(const Obstacle& other) {
    if (this != &other) {
        this->kind = other.kind;
        if (this->kind == Polygon) {
            this->value.polygon = other.value.polygon;
        } else {
            this->value.cylinder = other.value.cylinder;
        }
    }
    return *this;
}

// Victim
Victim::Victim(Point pos, float val) : position(pos), value(val) {}

// Map
Map::Map(Exit e, vector<Point> b, vector<Victim> v, vector<Obstacle> o) 
    : exit(e), borders(b), victims(v), obstacles(o) {}
