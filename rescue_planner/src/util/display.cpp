#include "display.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

void display(std::vector<std::tuple<Point, Point>> lines, std::vector<Point> points) {
    std::ostringstream ss;
    ss << lines.size() << " " << points.size() << "\n";

    for (const auto &t : lines) {
        const Point &a = std::get<0>(t);
        const Point &b = std::get<1>(t);
        ss << std::fixed << std::setprecision(6)
           << a.x << " " << a.y << " " << b.x << " " << b.y << "\n";
    }

    for (const auto &p : points) {
        ss << std::fixed << std::setprecision(6) << p.x << " " << p.y << "\n";
    }

    // Script path is relative to project root; adjust if you run the binary from a different CWD
    const std::string script = "display.py";
    std::string cmd = std::string("python3 \"") + script + "\"";

    FILE *pipe = popen(cmd.c_str(), "w");
    if (!pipe) {
        std::cerr << "display: failed to start python script '" << script << "'\n";
        return;
    }

    std::string out = ss.str();
    fwrite(out.data(), 1, out.size(), pipe);

    int rc = pclose(pipe);
    if (rc == -1) {
        std::cerr << "display: error when closing python pipe\n";
    }
}
