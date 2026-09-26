#pragma once

#include <imgui.h>

namespace app::ui {

// Result of interacting with the ViewCube: a requested camera orientation.
struct ViewCubeResult {
    bool clicked = false;
    float yaw = 0.0f;   // radians
    float pitch = 0.0f; // radians
};

// A CAD-style (Fusion 360 / Onshape style) navigation cube, drawn entirely
// with ImGui draw lists (no GL). World is +z up; FRONT faces -x (upstream,
// the car's nose), matching render::OrbitCamera's yaw/pitch convention:
//   eye = target + distance * (cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch))
//
// Face      Direction   yaw, pitch
//   FRONT     -x           180 deg, 0
//   BACK      +x             0 deg, 0
//   LEFT      -y           -90 deg, 0
//   RIGHT     +y            90 deg, 0
//   TOP       +z             *, +89 deg
//   BOTTOM    -z             *, -89 deg
class ViewCube {
public:
    // Face indices used by regionView()/hitTestCell().
    enum Face { FRONT = 0, BACK = 1, LEFT = 2, RIGHT = 3, TOP = 4, BOTTOM = 5 };

    // Draws the cube centred at `centre` (screen pixels), `size` px across,
    // oriented to match the camera (cameraYaw/cameraPitch, radians). Also
    // draws a home button, +/-90 deg rotate arrows and an axis triad. Returns
    // a target view when a region/button was clicked this frame.
    ViewCubeResult draw(ImVec2 centre, float size, float cameraYaw, float cameraPitch);

    // True while the mouse is over the widget (cube or helper buttons); the
    // app should not start an orbit/pan drag in that case.
    bool wantsMouse() const { return wantsMouse_; }

    // ---- Pure helpers, no ImGui dependency: usable directly from tests. ----

    // Maps one of a face's 3x3 cells (cellX, cellY each in {0,1,2}, column
    // then row) to the view it represents. The centre cell (1,1) is the face
    // itself; edge cells are the 45 deg view between two faces; corner cells
    // are the isometric corner view (pitch = +-35.264 deg). Shared edge/corner
    // cells on adjacent faces resolve to the same yaw/pitch. TOP/BOTTOM's
    // centre cell has no natural yaw, so it snaps `cameraYaw` to the nearest
    // 90 degrees instead of spinning the view.
    static ViewCubeResult regionView(int face, int cellX, int cellY, float cameraYaw);

    // Hit-tests a point in the widget's local space (i.e. mouse position minus
    // the cube's screen centre, in pixels) against the same projection draw()
    // uses. Only faces facing the camera (as draw() would render them) are
    // tested. Returns true and fills outFace/outCellX/outCellY on a hit.
    static bool hitTestCell(float localX, float localY, float size, float cameraYaw, float cameraPitch,
                             int& outFace, int& outCellX, int& outCellY);

private:
    ImVec2 centre_{0.0f, 0.0f};
    float size_ = 110.0f;
    bool wantsMouse_ = false;
};

} // namespace app::ui
