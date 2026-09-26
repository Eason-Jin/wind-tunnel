#pragma once

#include <optional>
#include <string>
#include <vector>

namespace app {

// Command-line options. Everything is optional; with no arguments the app
// opens a window showing a test sphere in the synthetic flow field.
//
//   --stl <file>            body to load (default: built-in sphere)
//   --scale <s>             STL units -> metres (e.g. 0.001 for mm), default 1
//   --up <z|y|x>            which STL axis points up (default z)
//   --yaw <deg>             rotate the body about the vertical axis
//   --pitch <deg>           rotate the body about the Y axis (e.g. 90 to lay a tall model down)
//   --tool <move|rotate>    start with the move or rotate gizmo active
//   --quality <draft|normal|fine>  simulation quality preset (default normal)
//   --field <spec>          synthetic | none | openfoam:<workDir>   (default synthetic)
//   --solve <openfoam|lbm>  run the OpenFOAM or GPU (lattice-Boltzmann) solver on startup
//   --passes <a,b,...>      enabled passes by name (case-insensitive), e.g. model,slice
//   --screenshot <out.png>  render offscreen, save PNG, exit (no window shown)
//   --frames <n>            frames to simulate before the screenshot (default 1)
//   --play                  start with the flow animation playing (after --solve finishes, if given)
//   --ui                    include the ImGui interface in the screenshot (use --frames >= 3)
//   --size <WxH>            screenshot / window size (default 1600x900)
//   --camera <yaw,pitch,zoom>  degrees, degrees, zoom factor (1 = fit body)
struct Options {
    std::optional<std::string> stlPath;
    float stlScale = 1.0f;
    int upAxis = 0;   // 0 = +z, 1 = +y, 2 = +x is up in the STL
    float yawDeg = 0.0f;   // rotation about the vertical axis
    float pitchDeg = 0.0f; // rotation about Y
    int tool = 0;          // 0 none, 1 move, 2 rotate
    int quality = 1;       // 0 draft, 1 normal, 2 fine
    std::string field = "synthetic";
    std::optional<std::string> solve;
    std::optional<std::vector<std::string>> passes;
    std::optional<std::string> screenshot;
    int frames = 1;
    bool showUi = false;
    bool play = false;
    int width = 1600, height = 900;
    std::optional<float> camYawDeg, camPitchDeg;
    float zoom = 1.0f;
};

// Parses argv; throws std::runtime_error with a usage message on bad input.
Options parseOptions(int argc, char** argv);

} // namespace app
