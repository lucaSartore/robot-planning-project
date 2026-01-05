#include <map>
#include <cmath>
#include <algorithm>
#include <tuple>
#include "trajectory_planner.hpp"
#include "../interface/interface.hpp"

// OccupationApproximation is a class that memorize the points on a map that are "occupated"
// (i.e.) the robot can't be in it by discretizing the space and creating a boolean map

std::tuple<int,int> OccupationApproximation::get_indexes(float x, float y) const {
    float fx = (x - x_min) / (x_max - x_min);
    float fy = (y - y_min) / (y_max - y_min);

    if (std::isnan(fx) || std::isinf(fx)) fx = 0.0f;
    if (std::isnan(fy) || std::isinf(fy)) fy = 0.0f;

    fx = std::max(0.0f, std::min(1.0f, fx));
    fy = std::max(0.0f, std::min(1.0f, fy));

    int ix = static_cast<int>(std::round(fx * (resolution_x - 1)));
    int iy = static_cast<int>(std::round(fy * (resolution_y - 1)));

    return {ix, iy};

}

bool OccupationApproximation::get(float x, float y) const {
    auto indexes = get_indexes(x, y);
    int ix = std::get<0>(indexes);
    int iy = std::get<1>(indexes);

    if (ix < 0 || ix >= resolution_x || iy < 0 || iy >= resolution_y) {
        return true; // out of bounds considered occupied
    }

    return this->ocupations[ix][iy];
}
bool OccupationApproximation::get(Point point) const {
    return get(point.x, point.y);
}


void OccupationApproximation::set(float x, float y, bool vlaue) {
    auto indexes = get_indexes(x, y);
    int ix = std::get<0>(indexes);
    int iy = std::get<1>(indexes);
    if (ix < 0 || ix >= resolution_x || iy < 0 || iy >= resolution_y) return;
    this->ocupations[ix][iy] = vlaue;
}
void OccupationApproximation::set(Point point, bool vlaue) {
    set(point.x, point.y, vlaue);
}

vector<vector<bool>> OccupationApproximation::get_kernel(int radius) {
    int size = 2*radius + 1;
    vector<vector<bool>> kernel(size, vector<bool>(size, false));
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int dx = i - radius;
            int dy = j - radius;
            if (dx*dx + dy*dy <= radius*radius) kernel[i][j] = true;
        }
    }
    return kernel;
}



void OccupationApproximation::draw_line(Point start, Point end) {
    auto si = get_indexes(start.x, start.y);
    auto ei = get_indexes(end.x, end.y);
    int x0 = std::get<0>(si);
    int y0 = std::get<1>(si);
    int x1 = std::get<0>(ei);
    int y1 = std::get<1>(ei);

    // Bresenham's line algorithm
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (x0 >= 0 && x0 < resolution_x && y0 >= 0 && y0 < resolution_y) {
            this->ocupations[x0][y0] = true;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2*err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
void OccupationApproximation::draw_circle(Point center, float radius) {
    float scale_x = static_cast<float>(resolution_x) / (x_max - x_min);
    float scale_y = static_cast<float>(resolution_y) / (y_max - y_min);
    float scale = (scale_x + scale_y) * 0.5f;
    int rpx = std::max(1, static_cast<int>(std::round(radius * scale)));

    auto ci = get_indexes(center.x, center.y);
    int cx = std::get<0>(ci);
    int cy = std::get<1>(ci);

    int xmin = std::max(0, cx - rpx);
    int xmax = std::min(resolution_x - 1, cx + rpx);
    int ymin = std::max(0, cy - rpx);
    int ymax = std::min(resolution_y - 1, cy + rpx);

    int r2 = rpx * rpx;
    for (int i = xmin; i <= xmax; ++i) {
        for (int j = ymin; j <= ymax; ++j) {
            int dx = i - cx;
            int dy = j - cy;
            if (dx*dx + dy*dy <= r2) this->ocupations[i][j] = true;
        }
    }
}

void OccupationApproximation::draw_map(Map const& map) {

    // draw the line and circles in the map

    // note: polygons and circles are drawn as outlines and not filled
    // because we will dilate them later

    // borders
    const vector<Point>& borders = map.borders;
    if (borders.size() >= 2) {
        for (size_t i = 0; i < borders.size(); ++i) {
            Point a = borders[i];
            Point b = borders[(i+1) % borders.size()];
            draw_line(a, b);
        }
    }

    // obstacles
    for (const auto &o: map.obstacles) {
        if (o.kind == Polygon) {
            const vector<Point>& pts = o.polygon.points;
            if (pts.size() >= 2) {
                for (size_t i = 0; i < pts.size(); ++i) {
                    Point a = pts[i];
                    Point b = pts[(i+1) % pts.size()];
                    draw_line(a, b);
                }
            }
        } else if (o.kind == Cylinder) {
            draw_circle(o.cylinder.center, o.cylinder.radius);
        }
    }
}

vector<vector<bool>> OccupationApproximation::dilation(vector<vector<bool>> const& source, vector<vector<bool>> const& kernel) {
    int sx = static_cast<int>(source.size());
    int sy = sx ? static_cast<int>(source[0].size()) : 0;
    int kx = static_cast<int>(kernel.size());
    int ky = kx ? static_cast<int>(kernel[0].size()) : 0;
    int krx = kx / 2;
    int kry = ky / 2;

    vector<vector<bool>> result = source;

    for (int i = 0; i < sx; ++i) {
        for (int j = 0; j < sy; ++j) {
            if (!source[i][j]) continue;
            for (int ki = 0; ki < kx; ++ki) {
                for (int kj = 0; kj < ky; ++kj) {
                    if (!kernel[ki][kj]) continue;
                    int tx = i + (ki - krx);
                    int ty = j + (kj - kry);
                    if (tx >= 0 && tx < sx && ty >= 0 && ty < sy) {
                        result[tx][ty] = true;
                    }
                }
            }
        }
    }

    return result;
}

OccupationApproximation::OccupationApproximation(Map const &map, int resolution_x, float radius, float margins) {

    vector<float> x_values = {};
    vector<float> y_values = {};
    for (auto p: map.borders) {
        x_values.push_back(p.x);
        y_values.push_back(p.y);
    }

    this->x_max = *std::max_element(x_values.begin(), x_values.end()) + margins;
    this->y_max = *std::max_element(y_values.begin(), y_values.end()) + margins;
    this->x_min = *std::min_element(x_values.begin(), x_values.end()) - margins;
    this->y_min = *std::min_element(y_values.begin(), y_values.end()) - margins;

    int resolution_y = static_cast<int>(std::round(static_cast<float>(resolution_x) * (y_max - y_min) / (x_max - x_min)));

    this->resolution_x = resolution_x;
    this->resolution_y = resolution_y;


    this->ocupations = {};
    this->ocupations.reserve(resolution_x);
    for (int i=0; i<resolution_x; i++) {
        vector<bool> x = {};
        x.reserve(resolution_y);
        for (int j=0; j<resolution_y; j++) {
            x.push_back(false);
        }
        this->ocupations.push_back(x);
    }

    // putting the elements in the map
    draw_map(map);

    // generate kernel for dilation
    int radius_in_pixels = static_cast<int>(std::round(radius / (x_max - x_min) * static_cast<float>(this->resolution_x)));
    radius_in_pixels = std::max(0, radius_in_pixels);
    auto kernel = get_kernel(radius_in_pixels);

    this->ocupations = dilation(this->ocupations, kernel);
}

bool OccupationApproximation::is_available(Point point) const {
    return !this->get(point);
}

