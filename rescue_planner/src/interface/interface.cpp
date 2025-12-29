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
    o.polygon = PolygonObstacle(pts);
    return o;
}

Obstacle Obstacle::CreateCylinder(Point center, float radius) {
    Obstacle o;
    o.kind = Cylinder;
    o.cylinder = CylinderObstacle(center, radius);
    return o;
}



// Victim
Victim::Victim(Point pos, float val) : position(pos), value(val) {}

// Map
Map::Map(Exit e, vector<Point> b, vector<Victim> v, vector<Obstacle> o) 
    : exit(e), borders(b), victims(v), obstacles(o) {}


ostream& operator<<(ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

ostream& operator<<(ostream& os, const Exit& e) {
    os << "Exit[Pos: " << e.position << ", Orient: " << e.orientation << "rad]";
    return os;
}

ostream& operator<<(ostream& os, const Obstacle& o) {
    os << "Obstacle(";
    if (o.kind == ObstacleKind::Polygon) {
        os << "Polygon: [";
        for (size_t i = 0; i < o.polygon.points.size(); ++i) {
            os << o.polygon.points[i] << (i == o.polygon.points.size() - 1 ? "" : ", ");
        }
        os << "]";
    } else if (o.kind == ObstacleKind::Cylinder) {
        os << "Cylinder: Center=" << o.cylinder.center 
           << ", Radius=" << o.cylinder.radius;
    }
    os << ")";
    return os;
}

ostream& operator<<(ostream& os, const Victim& v) {
    os << "Victim[Pos: " << v.position << ", Value: " << v.value << "]";
    return os;
}

ostream& operator<<(ostream& os, const Map& m) {
    os << "--- Map Report ---\n";
    os << m.exit << "\n";
    
    os << "Borders: ";
    for (const auto& p : m.borders) os << p << " ";
    os << "\n";

    os << "Victims (" << m.victims.size() << "):\n";
    for (const auto& v : m.victims) os << "  - " << v << "\n";

    os << "Obstacles (" << m.obstacles.size() << "):\n";
    for (const auto& obs : m.obstacles) os << "  - " << obs << "\n";
    
    os << "------------------";
    return os;
}
