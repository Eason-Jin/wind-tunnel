#include "app/ui/ViewCube.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace app::ui {

namespace {

constexpr float kCubeScaleFactor = 0.30f; // cube half-extent, in px, per `size`
constexpr float kIsoPitch = 0.615479709f; // asin(1/sqrt(3)), ~35.264 deg

struct FaceAxes {
    int normalAxis; // 0=x,1=y,2=z
    float sign;
    int axisU;
    int axisV;
};

FaceAxes faceAxes(int face)
{
    switch (face) {
        case ViewCube::FRONT: return {0, -1.0f, 1, 2};
        case ViewCube::BACK: return {0, 1.0f, 1, 2};
        case ViewCube::LEFT: return {1, -1.0f, 0, 2};
        case ViewCube::RIGHT: return {1, 1.0f, 0, 2};
        case ViewCube::TOP: return {2, 1.0f, 0, 1};
        case ViewCube::BOTTOM:
        default: return {2, -1.0f, 0, 1};
    }
}

const char* faceName(int face)
{
    switch (face) {
        case ViewCube::FRONT: return "FRONT";
        case ViewCube::BACK: return "BACK";
        case ViewCube::LEFT: return "LEFT";
        case ViewCube::RIGHT: return "RIGHT";
        case ViewCube::TOP: return "TOP";
        case ViewCube::BOTTOM:
        default: return "BOTTOM";
    }
}

// Point on a face at parametric (a,b) in [0,1]x[0,1], in local cube space
// (unit half-extent).
glm::vec3 facePoint(const FaceAxes& f, float a, float b)
{
    glm::vec3 p(0.0f);
    p[f.normalAxis] = f.sign;
    p[f.axisU] = -1.0f + 2.0f * a;
    p[f.axisV] = -1.0f + 2.0f * b;
    return p;
}

glm::vec3 faceNormal(const FaceAxes& f)
{
    glm::vec3 n(0.0f);
    n[f.normalAxis] = f.sign;
    return n;
}

// Camera basis matching render::OrbitCamera's convention (world +z up).
void cameraBasis(float yaw, float pitch, glm::vec3& eyeDir, glm::vec3& right, glm::vec3& up)
{
    pitch = std::clamp(pitch, glm::radians(-89.9f), glm::radians(89.9f));
    const float cp = std::cos(pitch);
    eyeDir = glm::vec3(cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch));
    const glm::vec3 forward = -eyeDir;
    const glm::vec3 worldUp(0.0f, 0.0f, 1.0f);
    right = glm::normalize(glm::cross(forward, worldUp));
    up = glm::cross(right, forward);
}

float nearest90(float yawRad)
{
    const float step = glm::half_pi<float>();
    return std::round(yawRad / step) * step;
}

// Signed sign helper that snaps a value at the exact grid coordinate (0 for
// the middle column/row) to precisely 0.
float cellAxisSign(int cell) { return static_cast<float>(cell - 1); } // -1, 0, +1

// 2D point-in-convex-quad test (quad given as 4 points, either winding).
bool pointInQuad(const ImVec2& p, const std::array<ImVec2, 4>& q)
{
    int pos = 0, neg = 0;
    for (int i = 0; i < 4; ++i) {
        const ImVec2& a = q[i];
        const ImVec2& b = q[(i + 1) % 4];
        const float cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if (cross > 0)
            ++pos;
        else if (cross < 0)
            ++neg;
    }
    return pos == 0 || neg == 0;
}

} // namespace

ViewCubeResult ViewCube::regionView(int face, int cellX, int cellY, float cameraYaw)
{
    const FaceAxes f = faceAxes(face);
    glm::vec3 dir(0.0f);
    dir[f.normalAxis] = f.sign;
    dir[f.axisU] = cellAxisSign(cellX);
    dir[f.axisV] = cellAxisSign(cellY);

    ViewCubeResult result;
    result.clicked = true;

    // TOP/BOTTOM's centre cell is the only direction with no x/y component,
    // so yaw is undefined; keep the current yaw snapped to the nearest
    // quarter turn instead of spinning to an arbitrary value.
    if (std::abs(dir.x) < 1e-6f && std::abs(dir.y) < 1e-6f) {
        result.yaw = nearest90(cameraYaw);
        result.pitch = glm::radians(dir.z > 0.0f ? 89.0f : -89.0f);
        return result;
    }

    const glm::vec3 n = glm::normalize(dir);
    result.yaw = std::atan2(n.y, n.x);
    result.pitch = std::clamp(std::asin(std::clamp(n.z, -1.0f, 1.0f)), glm::radians(-89.0f), glm::radians(89.0f));
    return result;
}

bool ViewCube::hitTestCell(float localX, float localY, float size, float cameraYaw, float cameraPitch, int& outFace,
                            int& outCellX, int& outCellY)
{
    glm::vec3 eyeDir, right, up;
    cameraBasis(cameraYaw, cameraPitch, eyeDir, right, up);
    const float scale = size * kCubeScaleFactor;
    const ImVec2 p{localX, localY};

    bool found = false;
    float bestDepth = -1.0f;
    for (int face = 0; face < 6; ++face) {
        const FaceAxes f = faceAxes(face);
        const glm::vec3 n = faceNormal(f);
        const float depth = glm::dot(n, eyeDir);
        if (depth <= 0.02f)
            continue; // back-facing: draw() doesn't render this face either

        for (int cy = 0; cy < 3; ++cy) {
            for (int cx = 0; cx < 3; ++cx) {
                const float a0 = cx / 3.0f, a1 = (cx + 1) / 3.0f;
                const float b0 = cy / 3.0f, b1 = (cy + 1) / 3.0f;
                std::array<ImVec2, 4> quad;
                const std::array<glm::vec2, 4> params{
                    glm::vec2{a0, b0}, glm::vec2{a1, b0}, glm::vec2{a1, b1}, glm::vec2{a0, b1}};
                for (int i = 0; i < 4; ++i) {
                    const glm::vec3 wp = facePoint(f, params[i].x, params[i].y);
                    quad[i] = ImVec2(scale * glm::dot(wp, right), -scale * glm::dot(wp, up));
                }
                if (pointInQuad(p, quad) && depth > bestDepth) {
                    bestDepth = depth;
                    outFace = face;
                    outCellX = cx;
                    outCellY = cy;
                    found = true;
                }
            }
        }
    }
    return found;
}

ViewCubeResult ViewCube::draw(ImVec2 centre, float size, float cameraYaw, float cameraPitch)
{
    centre_ = centre;
    size_ = size;
    ViewCubeResult result;

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 mouse = io.MousePos;
    const ImVec2 local(mouse.x - centre.x, mouse.y - centre.y);

    // Overall widget bounding box (cube + home button + rotate arrows).
    const float hitRadius = size * 0.85f;
    wantsMouse_ = std::abs(local.x) < hitRadius && std::abs(local.y) < hitRadius;

    const bool mouseClicked = ImGui::IsMouseClicked(ImGuiMouseButton_Left);

    glm::vec3 eyeDir, right, up;
    cameraBasis(cameraYaw, cameraPitch, eyeDir, right, up);
    const float scale = size * kCubeScaleFactor;
    auto project = [&](const glm::vec3& p) {
        return ImVec2(centre.x + scale * glm::dot(p, right), centre.y - scale * glm::dot(p, up));
    };

    int hoverFace = -1, hoverCellX = -1, hoverCellY = -1;
    const bool hasHover = hitTestCell(local.x, local.y, size, cameraYaw, cameraPitch, hoverFace, hoverCellX, hoverCellY);

    // Faces sorted back-to-front so nearer faces paint over farther ones at
    // shared silhouette edges.
    std::array<int, 6> order{0, 1, 2, 3, 4, 5};
    std::array<float, 6> depth{};
    for (int face = 0; face < 6; ++face)
        depth[face] = glm::dot(faceNormal(faceAxes(face)), eyeDir);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return depth[a] < depth[b]; });

    // Opaque slate faces that match the dark UI, so the cube reads clearly
    // over busy flow visuals; the 3x3 regions only show up on hover.
    const ImU32 faceCol = IM_COL32(52, 60, 72, 240);
    const ImU32 faceHoverCol = IM_COL32(61, 139, 253, 245);
    const ImU32 cellLineCol = IM_COL32(255, 255, 255, 14);
    const ImU32 edgeCol = IM_COL32(150, 166, 188, 220);
    const ImU32 iconCol = IM_COL32(232, 238, 246, 255);

    for (int face : order) {
        if (depth[face] <= 0.02f)
            continue; // back-facing
        const FaceAxes f = faceAxes(face);

        for (int cy = 0; cy < 3; ++cy) {
            for (int cx = 0; cx < 3; ++cx) {
                const float a0 = cx / 3.0f, a1 = (cx + 1) / 3.0f;
                const float b0 = cy / 3.0f, b1 = (cy + 1) / 3.0f;
                const ImVec2 p00 = project(facePoint(f, a0, b0));
                const ImVec2 p10 = project(facePoint(f, a1, b0));
                const ImVec2 p11 = project(facePoint(f, a1, b1));
                const ImVec2 p01 = project(facePoint(f, a0, b1));

                const bool hovered = hasHover && hoverFace == face && hoverCellX == cx && hoverCellY == cy;
                dl->AddQuadFilled(p00, p10, p11, p01, hovered ? faceHoverCol : faceCol);
                dl->AddQuad(p00, p10, p11, p01, cellLineCol, 1.0f);
            }
        }

        // Label the face on its centre cell only.
        const ImVec2 c00 = project(facePoint(f, 0.0f, 0.0f));
        const ImVec2 c10 = project(facePoint(f, 1.0f, 0.0f));
        const ImVec2 c11 = project(facePoint(f, 1.0f, 1.0f));
        const ImVec2 c01 = project(facePoint(f, 0.0f, 1.0f));
        const ImVec2 faceCentre((c00.x + c10.x + c11.x + c01.x) * 0.25f, (c00.y + c10.y + c11.y + c01.y) * 0.25f);
        const float faceSpan = std::max(1.0f, std::hypot(c10.x - c00.x, c10.y - c00.y));
        dl->AddQuad(c00, c10, c11, c01, edgeCol, 1.5f);
        const float fontSize = std::clamp(faceSpan * 0.2f, 10.0f, 15.0f);
        const char* label = faceName(face);
        // Fade labels on faces seen edge-on, where they would be squashed.
        const float facing = std::clamp((depth[face] - 0.15f) / 0.35f, 0.0f, 1.0f);
        const ImU32 textCol = IM_COL32(232, 238, 246, static_cast<int>(255 * facing));
        ImGui::PushFont(nullptr, fontSize);
        const ImVec2 textSize = ImGui::CalcTextSize(label);
        ImGui::PopFont();
        dl->AddText(ImGui::GetFont(), fontSize, ImVec2(faceCentre.x - textSize.x * 0.5f, faceCentre.y - textSize.y * 0.5f),
                    textCol, label);
    }

    if (hasHover && mouseClicked)
        result = regionView(hoverFace, hoverCellX, hoverCellY, cameraYaw);

    // ---- Home button (isometric FRONT-LEFT-TOP corner). ----
    const ImVec2 homeCentre(centre.x - size * 0.62f, centre.y + size * 0.55f);
    const float homeRadius = size * 0.11f;
    const bool homeHover = std::hypot(mouse.x - homeCentre.x, mouse.y - homeCentre.y) <= homeRadius;
    dl->AddCircleFilled(homeCentre, homeRadius, homeHover ? IM_COL32(120, 170, 230, 230) : IM_COL32(90, 100, 112, 200));
    dl->AddCircle(homeCentre, homeRadius, edgeCol, 0, 1.2f);
    // Simple house glyph: a triangle roof over a square base.
    const float hr = homeRadius * 0.55f;
    dl->AddTriangle(ImVec2(homeCentre.x - hr, homeCentre.y), ImVec2(homeCentre.x + hr, homeCentre.y),
                     ImVec2(homeCentre.x, homeCentre.y - hr * 1.3f), iconCol, 1.5f);
    dl->AddRect(ImVec2(homeCentre.x - hr * 0.7f, homeCentre.y), ImVec2(homeCentre.x + hr * 0.7f, homeCentre.y + hr),
                iconCol, 0.0f, 0, 1.5f);
    if (homeHover && mouseClicked) {
        result.clicked = true;
        const glm::vec3 homeDir = glm::normalize(glm::vec3(-1.0f, -1.0f, 1.0f));
        result.yaw = std::atan2(homeDir.y, homeDir.x);
        result.pitch = kIsoPitch;
    }

    // ---- +/-90 deg rotate arrows, either side of the cube. ----
    const ImVec2 leftArrow(centre.x - size * 0.62f, centre.y - size * 0.05f);
    const ImVec2 rightArrow(centre.x + size * 0.62f, centre.y - size * 0.05f);
    const float arrowRadius = size * 0.09f;
    const bool leftHover = std::hypot(mouse.x - leftArrow.x, mouse.y - leftArrow.y) <= arrowRadius;
    const bool rightHover = std::hypot(mouse.x - rightArrow.x, mouse.y - rightArrow.y) <= arrowRadius;
    dl->AddCircleFilled(leftArrow, arrowRadius, leftHover ? IM_COL32(120, 170, 230, 230) : IM_COL32(90, 100, 112, 200));
    dl->AddCircleFilled(rightArrow, arrowRadius, rightHover ? IM_COL32(120, 170, 230, 230) : IM_COL32(90, 100, 112, 200));
    dl->AddCircle(leftArrow, arrowRadius, edgeCol, 0, 1.2f);
    dl->AddCircle(rightArrow, arrowRadius, edgeCol, 0, 1.2f);
    {
        const char* lLabel = "<";
        const char* rLabel = ">";
        const float afs = arrowRadius * 1.1f;
        ImGui::PushFont(nullptr, afs);
        const ImVec2 lts = ImGui::CalcTextSize(lLabel);
        const ImVec2 rts = ImGui::CalcTextSize(rLabel);
        ImGui::PopFont();
        dl->AddText(ImGui::GetFont(), afs, ImVec2(leftArrow.x - lts.x * 0.5f, leftArrow.y - lts.y * 0.5f), iconCol, lLabel);
        dl->AddText(ImGui::GetFont(), afs, ImVec2(rightArrow.x - rts.x * 0.5f, rightArrow.y - rts.y * 0.5f), iconCol, rLabel);
    }
    if (mouseClicked && (leftHover || rightHover)) {
        result.clicked = true;
        result.yaw = cameraYaw + (leftHover ? -glm::half_pi<float>() : glm::half_pi<float>());
        result.pitch = cameraPitch;
    }

    // ---- Axis triad, lower-left of the cube. ----
    const ImVec2 triadOrigin(centre.x - size * 0.62f, centre.y + size * 0.20f);
    const float triadLen = size * 0.16f;
    const glm::vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const ImU32 axisCol[3] = {IM_COL32(220, 70, 70, 255), IM_COL32(70, 200, 90, 255), IM_COL32(70, 120, 230, 255)};
    const char* axisLabel[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        const ImVec2 tip(triadOrigin.x + triadLen * glm::dot(axes[i], right), triadOrigin.y - triadLen * glm::dot(axes[i], up));
        dl->AddLine(triadOrigin, tip, axisCol[i], 2.0f);
        dl->AddCircleFilled(tip, 2.5f, axisCol[i]);
        dl->AddText(ImVec2(tip.x + 3.0f, tip.y - 6.0f), axisCol[i], axisLabel[i]);
    }

    return result;
}

} // namespace app::ui
