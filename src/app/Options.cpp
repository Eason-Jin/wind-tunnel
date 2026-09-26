#include "app/Options.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace app {

namespace {

std::vector<std::string> split(const std::string& s, char sep)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, sep))
        if (!item.empty())
            out.push_back(item);
    return out;
}

} // namespace

Options parseOptions(int argc, char** argv)
{
    Options o;
    auto need = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error(std::string("Missing value for ") + argv[i]);
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--stl")
            o.stlPath = need(i);
        else if (a == "--scale")
            o.stlScale = std::stof(need(i));
        else if (a == "--up") {
            const std::string v = need(i);
            if (v == "z" || v == "Z")
                o.upAxis = 0;
            else if (v == "y" || v == "Y")
                o.upAxis = 1;
            else if (v == "x" || v == "X")
                o.upAxis = 2;
            else
                throw std::runtime_error("--up expects z, y or x");
        } else if (a == "--yaw")
            o.yawSteps = ((static_cast<int>(std::lround(std::stof(need(i)) / 90.0f)) % 4) + 4) % 4;
        else if (a == "--field")
            o.field = need(i);
        else if (a == "--solve")
            o.solve = need(i);
        else if (a == "--passes")
            o.passes = split(need(i), ',');
        else if (a == "--play")
            o.play = true;
        else if (a == "--ui")
            o.showUi = true;
        else if (a == "--screenshot")
            o.screenshot = need(i);
        else if (a == "--frames")
            o.frames = std::max(1, std::stoi(need(i)));
        else if (a == "--size") {
            const std::string v = need(i);
            if (std::sscanf(v.c_str(), "%dx%d", &o.width, &o.height) != 2 || o.width <= 0 || o.height <= 0)
                throw std::runtime_error("--size expects WxH, got " + v);
        } else if (a == "--camera") {
            const auto parts = split(need(i), ',');
            if (parts.size() != 3)
                throw std::runtime_error("--camera expects yaw,pitch,zoom");
            o.yawDeg = std::stof(parts[0]);
            o.pitchDeg = std::stof(parts[1]);
            o.zoom = std::stof(parts[2]);
        } else if (a == "--help" || a == "-h") {
            throw std::runtime_error("usage: windtunnel [--stl file] [--scale s] [--up z|y|x] [--yaw deg] [--field synthetic|none|openfoam:<dir>]\n"
                                     "  [--solve openfoam] [--passes a,b] [--screenshot out.png] [--frames n] [--ui] [--play]\n"
                                     "  [--size WxH] [--camera yaw,pitch,zoom]");
        } else {
            throw std::runtime_error("Unknown option: " + a + " (try --help)");
        }
    }
    return o;
}

} // namespace app
