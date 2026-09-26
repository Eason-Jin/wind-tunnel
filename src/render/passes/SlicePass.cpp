#include "render/passes/SlicePass.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace render {

namespace {

constexpr GLuint kColormapUnit = 4;

struct LineVertex {
    glm::vec3 pos;
    glm::vec4 colour;
};

// Index of the plane's two in-plane axes (in the same order used for the
// quad corners and glyph sampling), given the normal axis.
std::pair<int, int> planeAxes(SlicePass::Axis axis)
{
    switch (axis) {
    case SlicePass::Axis::X:
        return {1, 2}; // y, z
    case SlicePass::Axis::Y:
        return {0, 2}; // x, z
    case SlicePass::Axis::Z:
        return {0, 1}; // x, y
    }
    return {0, 1};
}

glm::vec3 makePoint(SlicePass::Axis axis, float coord, float u, float v)
{
    glm::vec3 p(0.0f);
    const auto [ia, ib] = planeAxes(axis);
    p[static_cast<int>(axis)] = coord;
    p[ia] = u;
    p[ib] = v;
    return p;
}

const char* quantityName(SlicePass::Quantity q)
{
    switch (q) {
    case SlicePass::Quantity::Speed:
        return "Speed |U|";
    case SlicePass::Quantity::Cp:
        return "Pressure coeff. Cp";
    case SlicePass::Quantity::Ux:
        return "Velocity Ux";
    case SlicePass::Quantity::Vorticity:
        return "Vorticity |w|";
    }
    return "?";
}

const char* quantityUnits(SlicePass::Quantity q)
{
    switch (q) {
    case SlicePass::Quantity::Speed:
    case SlicePass::Quantity::Ux:
        return "m/s";
    case SlicePass::Quantity::Vorticity:
        return "1/s";
    case SlicePass::Quantity::Cp:
        return "";
    }
    return "";
}

float scalarAtCell(const core::FlowField& f, int i, int j, int k, SlicePass::Quantity q)
{
    const std::size_t n = f.index(i, j, k);
    switch (q) {
    case SlicePass::Quantity::Speed:
        return glm::length(f.velocity[n]);
    case SlicePass::Quantity::Ux:
        return f.velocity[n].x;
    case SlicePass::Quantity::Cp:
        return f.pressure.empty() ? 0.0f : f.pressure[n] / std::max(0.5f * f.freestreamSpeed * f.freestreamSpeed, 1e-6f);
    case SlicePass::Quantity::Vorticity: {
        auto clampAxis = [](int x, int n) { return std::clamp(x, 0, n - 1); };
        const int ip = clampAxis(i + 1, f.dims.x), im = clampAxis(i - 1, f.dims.x);
        const int jp = clampAxis(j + 1, f.dims.y), jm = clampAxis(j - 1, f.dims.y);
        const int kp = clampAxis(k + 1, f.dims.z), km = clampAxis(k - 1, f.dims.z);
        const glm::vec3 vXp = f.velocity[f.index(ip, j, k)], vXm = f.velocity[f.index(im, j, k)];
        const glm::vec3 vYp = f.velocity[f.index(i, jp, k)], vYm = f.velocity[f.index(i, jm, k)];
        const glm::vec3 vZp = f.velocity[f.index(i, j, kp)], vZm = f.velocity[f.index(i, j, km)];
        const float dx = f.spacing.x * std::max(ip - im, 1);
        const float dy = f.spacing.y * std::max(jp - jm, 1);
        const float dz = f.spacing.z * std::max(kp - km, 1);
        const float dVzdy = (vYp.z - vYm.z) / dy, dVydz = (vZp.y - vZm.y) / dz;
        const float dVxdz = (vZp.x - vZm.x) / dz, dVzdx = (vXp.z - vXm.z) / dx;
        const float dVydx = (vXp.y - vXm.y) / dx, dVxdy = (vYp.x - vYm.x) / dy;
        const glm::vec3 curl(dVzdy - dVydz, dVxdz - dVzdx, dVydx - dVxdy);
        return glm::length(curl);
    }
    }
    return 0.0f;
}

} // namespace

SlicePass::SlicePass()
    : planeShader_(gl::Shader::fromFiles("slice.vert", "slice.frag")), lineShader_(gl::Shader::fromFiles("lines.vert", "lines.frag"))
{
    colormap_.set(static_cast<ColormapKind>(colormapIndex_));
    resetRangeForQuantity();
}

SlicePass::~SlicePass() = default;

void SlicePass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    updateDefaultPosition();
    rebuildPlane();
    rebuildGlyphs();
}

void SlicePass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    updateDefaultPosition();
    resetRangeForQuantity();
    rebuildPlane();
    rebuildGlyphs();
}

void SlicePass::update(const FrameContext&) {}

void SlicePass::updateDefaultPosition()
{
    if (userSetPosition_ || axis_ != Axis::Y)
        return;
    if (!scene_.field || scene_.field->empty())
        return;
    const core::Bounds b = scene_.field->bounds();
    const float centreY = (scene_.body && !scene_.body->empty()) ? scene_.body->bounds().centre().y : 0.5f;
    const float size = b.size().y;
    position_ = size > 1e-6f ? std::clamp((centreY - b.min.y) / size, 0.0f, 1.0f) : 0.5f;
}

void SlicePass::resetRangeForQuantity()
{
    const float maxSpeed = (scene_.field && !scene_.field->empty()) ? scene_.field->maxSpeed() : 1.0f;
    switch (quantity_) {
    case Quantity::Speed:
        rangeMin_ = 0.0f;
        rangeMax_ = std::max(maxSpeed, 1e-3f);
        break;
    case Quantity::Cp:
        rangeMin_ = -1.5f;
        rangeMax_ = 1.0f;
        break;
    case Quantity::Ux:
        rangeMin_ = -std::max(maxSpeed, 1e-3f);
        rangeMax_ = std::max(maxSpeed, 1e-3f);
        break;
    case Quantity::Vorticity:
        rangeMin_ = 0.0f;
        rangeMax_ = 1.0f;
        if (scene_.field && !scene_.field->empty())
            computeAutoRange();
        break;
    }
}

void SlicePass::computeAutoRange()
{
    if (!scene_.field || scene_.field->empty())
        return;
    const core::FlowField& f = *scene_.field;
    float lo = std::numeric_limits<float>::max();
    float hi = std::numeric_limits<float>::lowest();
    for (int k = 0; k < f.dims.z; ++k)
        for (int j = 0; j < f.dims.y; ++j)
            for (int i = 0; i < f.dims.x; ++i) {
                if (!f.solid.empty() && f.solid[f.index(i, j, k)])
                    continue;
                const float v = scalarAtCell(f, i, j, k, quantity_);
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
    if (lo <= hi) {
        rangeMin_ = lo;
        rangeMax_ = hi;
    }
}

void SlicePass::rebuildPlane()
{
    quadVertexCount_ = 0;
    if (!scene_.field || scene_.field->empty())
        return;

    const core::Bounds b = scene_.field->bounds();
    const int axisIdx = static_cast<int>(axis_);
    const float coord = b.min[axisIdx] + position_ * b.size()[axisIdx];
    const auto [ia, ib] = planeAxes(axis_);

    const glm::vec3 p00 = makePoint(axis_, coord, b.min[ia], b.min[ib]);
    const glm::vec3 p10 = makePoint(axis_, coord, b.max[ia], b.min[ib]);
    const glm::vec3 p11 = makePoint(axis_, coord, b.max[ia], b.max[ib]);
    const glm::vec3 p01 = makePoint(axis_, coord, b.min[ia], b.max[ib]);
    const std::array<glm::vec3, 6> verts = {p00, p10, p11, p00, p11, p01};

    quadVertexCount_ = static_cast<int>(verts.size());
    quadVbo_ = gl::createBuffer();
    glNamedBufferStorage(quadVbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(glm::vec3)), verts.data(), 0);
    quadVao_ = gl::createVertexArray();
    glVertexArrayVertexBuffer(quadVao_.id(), 0, quadVbo_.id(), 0, sizeof(glm::vec3));
    glEnableVertexArrayAttrib(quadVao_.id(), 0);
    glVertexArrayAttribFormat(quadVao_.id(), 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribBinding(quadVao_.id(), 0, 0);
}

void SlicePass::rebuildGlyphs()
{
    glyphVertexCount_ = 0;
    if (!showGlyphs_ || !scene_.field || scene_.field->empty())
        return;

    const core::FlowField& f = *scene_.field;
    const core::Bounds b = f.bounds();
    const int axisIdx = static_cast<int>(axis_);
    const float coord = b.min[axisIdx] + position_ * b.size()[axisIdx];
    const auto [ia, ib] = planeAxes(axis_);
    const float uSize = b.max[ia] - b.min[ia];
    const float vSize = b.max[ib] - b.min[ib];
    const float longer = std::max(uSize, vSize);
    const int nu = std::max(2, static_cast<int>(std::round(glyphDensity_ * (uSize / std::max(longer, 1e-6f)))));
    const int nv = std::max(2, static_cast<int>(std::round(glyphDensity_ * (vSize / std::max(longer, 1e-6f)))));
    const float du = uSize / nu;
    const float dv = vSize / nv;
    const float maxLen = 0.4f * std::min(du, dv);
    const float maxSpeed = std::max(f.maxSpeed(), 1e-6f);

    std::vector<LineVertex> verts;
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            const float u = b.min[ia] + (i + 0.5f) * du;
            const float v = b.min[ib] + (j + 0.5f) * dv;
            const glm::vec3 p = makePoint(axis_, coord, u, v);
            if (!f.contains(p) || f.isSolid(p))
                continue;
            const glm::vec3 vel = f.sampleVelocity(p);
            const glm::vec2 velPlane(vel[ia], vel[ib]);
            const float speed = glm::length(vel);
            if (speed < 1e-4f)
                continue;
            const glm::vec2 dir = velPlane / std::max(glm::length(velPlane), 1e-6f);
            const float len = maxLen * std::clamp(speed / maxSpeed, 0.15f, 1.0f);
            const glm::vec4 colour = glyphColourBySpeed_
                                          ? glm::vec4(colormap(colormap_.kind(), std::clamp(speed / maxSpeed, 0.0f, 1.0f)), 1.0f)
                                          : glm::vec4(1.0f);

            const glm::vec3 tip = makePoint(axis_, coord, u + dir.x * len, v + dir.y * len);
            verts.push_back({p, colour});
            verts.push_back({tip, colour});

            const glm::vec2 perp(-dir.y, dir.x);
            const glm::vec3 barb1 = makePoint(axis_, coord, u + dir.x * len * 0.7f + perp.x * len * 0.25f,
                                              v + dir.y * len * 0.7f + perp.y * len * 0.25f);
            const glm::vec3 barb2 = makePoint(axis_, coord, u + dir.x * len * 0.7f - perp.x * len * 0.25f,
                                              v + dir.y * len * 0.7f - perp.y * len * 0.25f);
            verts.push_back({tip, colour});
            verts.push_back({barb1, colour});
            verts.push_back({tip, colour});
            verts.push_back({barb2, colour});
        }
    }

    if (verts.empty())
        return;

    glyphVertexCount_ = static_cast<int>(verts.size());
    glyphVbo_ = gl::createBuffer();
    glNamedBufferStorage(glyphVbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(LineVertex)), verts.data(), 0);
    glyphVao_ = gl::createVertexArray();
    glVertexArrayVertexBuffer(glyphVao_.id(), 0, glyphVbo_.id(), 0, sizeof(LineVertex));
    glEnableVertexArrayAttrib(glyphVao_.id(), 0);
    glVertexArrayAttribFormat(glyphVao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(LineVertex, pos));
    glVertexArrayAttribBinding(glyphVao_.id(), 0, 0);
    glEnableVertexArrayAttrib(glyphVao_.id(), 1);
    glVertexArrayAttribFormat(glyphVao_.id(), 1, 4, GL_FLOAT, GL_FALSE, offsetof(LineVertex, colour));
    glVertexArrayAttribBinding(glyphVao_.id(), 1, 0);
}

void SlicePass::draw(const FrameContext& frame)
{
    if (quadVertexCount_ == 0 || !scene_.flowTextures || !scene_.flowTextures->valid())
        return;

    const bool blending = opacity_ < 1.0f;
    if (blending) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    colormap_.bind(kColormapUnit);
    planeShader_.set("uViewProj", frame.proj * frame.view);
    scene_.flowTextures->setUniforms(planeShader_);
    planeShader_.set("uColormap", static_cast<int>(kColormapUnit));
    planeShader_.set("uQuantity", static_cast<int>(quantity_));
    planeShader_.set("uRangeMin", rangeMin_);
    planeShader_.set("uRangeMax", rangeMax_);
    planeShader_.set("uShowSolid", showSolid_ ? 1 : 0);
    planeShader_.set("uContours", showContours_ ? 1 : 0);
    planeShader_.set("uContourCount", contourCount_);
    planeShader_.set("uOpacity", opacity_);
    planeShader_.use();
    glBindVertexArray(quadVao_.id());
    glDrawArrays(GL_TRIANGLES, 0, quadVertexCount_);

    if (glyphVertexCount_ > 0) {
        // Glyphs sit exactly on the plane; GL_LEQUAL keeps them from losing
        // the depth test against the quad drawn just before them.
        glDepthFunc(GL_LEQUAL);
        lineShader_.set("uViewProj", frame.proj * frame.view);
        lineShader_.use();
        glBindVertexArray(glyphVao_.id());
        glDrawArrays(GL_LINES, 0, glyphVertexCount_);
        glDepthFunc(GL_LESS);
    }

    if (blending) {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

namespace {
bool axisCombo(const char* label, SlicePass::Axis& axis)
{
    static const char* names[] = {"X", "Y", "Z"};
    int idx = static_cast<int>(axis);
    if (ImGui::Combo(label, &idx, names, 3)) {
        axis = static_cast<SlicePass::Axis>(idx);
        return true;
    }
    return false;
}
} // namespace

void SlicePass::drawUi()
{
    bool planeChanged = false;

    if (axisCombo("Axis", axis_)) {
        userSetPosition_ = true;
        planeChanged = true;
    }
    if (ImGui::SliderFloat("Position", &position_, 0.0f, 1.0f)) {
        userSetPosition_ = true;
        planeChanged = true;
    }

    int quantityIdx = static_cast<int>(quantity_);
    static const char* quantityNames[] = {"Speed |U|", "Pressure Cp", "Velocity Ux", "Vorticity |w|"};
    if (ImGui::Combo("Quantity", &quantityIdx, quantityNames, 4)) {
        quantity_ = static_cast<Quantity>(quantityIdx);
        resetRangeForQuantity();
    }

    constexpr int kColormapCount = static_cast<int>(sizeof(kColormapNames) / sizeof(kColormapNames[0]));
    if (ImGui::Combo("Colormap", &colormapIndex_, kColormapNames, kColormapCount))
        colormap_.set(static_cast<ColormapKind>(colormapIndex_));

    ImGui::DragFloat("Range min", &rangeMin_, 0.01f);
    ImGui::DragFloat("Range max", &rangeMax_, 0.01f);
    if (ImGui::Button("Auto range"))
        computeAutoRange();

    ImGui::Checkbox("Show solid", &showSolid_);
    ImGui::Checkbox("Contour lines", &showContours_);
    if (showContours_) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::SliderInt("Levels", &contourCount_, 2, 30);
    }

    if (ImGui::Checkbox("Velocity glyphs", &showGlyphs_))
        rebuildGlyphs();
    if (showGlyphs_) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        if (ImGui::SliderInt("Density", &glyphDensity_, 4, 40))
            rebuildGlyphs();
        if (ImGui::Checkbox("Colour glyphs by speed", &glyphColourBySpeed_))
            rebuildGlyphs();
    }

    ImGui::SliderFloat("Opacity", &opacity_, 0.0f, 1.0f);

    if (planeChanged)
        rebuildPlane();
    if (planeChanged && showGlyphs_)
        rebuildGlyphs();

    // Colour bar.
    ImGui::Text("%s", quantityName(quantity_));
    const float barWidth = ImGui::GetContentRegionAvail().x;
    const float barHeight = 16.0f;
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    constexpr int kSteps = 64;
    for (int i = 0; i < kSteps; ++i) {
        const float t0 = static_cast<float>(i) / kSteps;
        const float t1 = static_cast<float>(i + 1) / kSteps;
        const glm::vec3 c = colormap(colormap_.kind(), t0);
        const ImVec2 a(p0.x + t0 * barWidth, p0.y);
        const ImVec2 bb(p0.x + t1 * barWidth, p0.y + barHeight);
        draw->AddRectFilled(a, bb, ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, 1.0f)));
    }
    draw->AddRect(p0, ImVec2(p0.x + barWidth, p0.y + barHeight), IM_COL32(0, 0, 0, 128));
    ImGui::Dummy(ImVec2(barWidth, barHeight));

    const char* units = quantityUnits(quantity_);
    if (units[0] != '\0')
        ImGui::Text("%.2f %s", rangeMin_, units);
    else
        ImGui::Text("%.2f", rangeMin_);
    ImGui::SameLine(barWidth - 60.0f);
    if (units[0] != '\0')
        ImGui::Text("%.2f %s", rangeMax_, units);
    else
        ImGui::Text("%.2f", rangeMax_);
}

} // namespace render
