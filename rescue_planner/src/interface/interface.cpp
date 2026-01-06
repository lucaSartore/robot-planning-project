#include "interface.hpp"

#include <complex>
#include <exception>
#include <math.h>

Map Interface::GetMap() {
    throw std::logic_error("GetMap(): not implemented");
};
void Interface::OutputTrajectory(vector<Point> trajectory) {};

Point::Point(float x, float y) : x(x), y(y) {}

Point Point::FromPolar(float angle, float radius) {
    float x = cos(angle) * radius;
    float y = sin(angle) * radius;
    return {x, y};
}


bool Point::operator ==(const Point point) const {
    return x == point.x && y == point.y;
}
void Point::operator +=(const Point point) {
    this->x += point.x;
    this->y += point.y;
}
Point Point::operator +(const Point point) const {
    return {x + point.x, y + point.y};
}

void Point::operator /=(const float v) {
    x/=v;
    y/=v;
}
Point Point::operator /(const float v) const {
    return {x/v,y/v};
}

Pose::Pose(Point pos, float orient) : position(pos), orientation(orient) {}

PolygonObstacle::PolygonObstacle(const vector<Point>& pts) : points(pts) {}

CylinderObstacle::CylinderObstacle(Point c, float r) : center(c), radius(r) {}

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


Victim::Victim(Point pos, float val) : position(pos), value(val) {}

Map::Map(Pose e, Pose r, vector<Point> b, vector<Victim> v, vector<Obstacle> o) 
    : exit(e), robot_position(r), borders(b), victims(v), obstacles(o) {}


ostream& operator<<(ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

ostream& operator<<(ostream& os, const Pose& e) {
    os << "Pose[Pos: " << e.position << ", Orient: " << e.orientation << "rad]";
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
    os << "Exit: " << m.exit << "\n";

    os << "RobotPosition: " << m.robot_position << "\n";
    
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

vector<tuple<Point, Point> > Map::get_obstacle_lines() {
    vector<tuple<Point, Point>> v;

    auto add = [&](vector<Point> p) {
        for (int i=0; i<p.size(); i++) {
            int j = (i+1)%p.size();
            v.push_back({p[i], p[j]});
        }
    };


    add(borders);
    for (auto & obstacle: obstacles) {
        if (obstacle.kind == ObstacleKind::Polygon) {
            add(obstacle.polygon.points);
        } else {
            vector<Point> circle_points = {};
            for (int i=0; i<=16; i++) {
                circle_points.push_back(
                    obstacle.cylinder.center + Point::FromPolar(
                        i * 2 * M_PI / 16, obstacle.cylinder.radius
                    )
                );
            }
            add(circle_points);
        }
    }
    return v;
}

vector<tuple<Point, Point> > Map::get_victims_lines() {
    vector<tuple<Point, Point>> to_return;



    for (auto & v: victims) {
        auto a = v.position + Point::FromPolar(M_PI/2, 0.3);
        auto b = v.position + Point::FromPolar(-M_PI/6, 0.3);
        auto c = v.position + Point::FromPolar(M_PI*7/6, 0.3);
        to_return.push_back({a, b});
        to_return.push_back({b, c});
        to_return.push_back({c, a});
    }
    return to_return;
}
