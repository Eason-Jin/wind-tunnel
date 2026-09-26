#include "render/passes/SlicePass.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

namespace render {

namespace {

constexpr GLuint kColormapUnit = 4;
constexpr GLuint kStreamColormapUnit = 5;

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

// --- Evenly-spaced in-plane streamlines (Jobard & Lefer 1997) ---------------
//
// A background occupancy grid over the plane's (u, v) rectangle records
// every placed streamline point. Seeding a new line checks the grid so lines
// stay roughly `dsep` apart; tracing a line stops early if it drifts within
// `dtest` (~dsep/2) of another line, which is what keeps the field from
// clumping instead of merely capping a fixed seed count.
class PlaneOccupancy {
public:
    PlaneOccupancy(glm::vec2 origin, glm::vec2 size, float cellSize) : origin_(origin), cell_(std::max(cellSize, 1e-5f))
    {
        nx_ = std::clamp(static_cast<int>(std::ceil(size.x / cell_)) + 2, 1, 1024);
        ny_ = std::clamp(static_cast<int>(std::ceil(size.y / cell_)) + 2, 1, 1024);
        cells_.resize(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_));
    }

    void insert(glm::vec2 p)
    {
        const auto [cx, cy] = cellOf(p);
        if (cx >= 0 && cx < nx_ && cy >= 0 && cy < ny_)
            cells_[idx(cx, cy)].push_back(p);
    }

    bool within(glm::vec2 p, float dist) const
    {
        const auto [cx, cy] = cellOf(p);
        const int r = std::max(1, static_cast<int>(std::ceil(dist / cell_)));
        for (int dy = -r; dy <= r; ++dy) {
            const int gy = cy + dy;
            if (gy < 0 || gy >= ny_)
                continue;
            for (int dx = -r; dx <= r; ++dx) {
                const int gx = cx + dx;
                if (gx < 0 || gx >= nx_)
                    continue;
                for (const glm::vec2& q : cells_[idx(gx, gy)])
                    if (glm::length(q - p) < dist)
                        return true;
            }
        }
        return false;
    }

private:
    std::pair<int, int> cellOf(glm::vec2 p) const
    {
        return {static_cast<int>(std::floor((p.x - origin_.x) / cell_)), static_cast<int>(std::floor((p.y - origin_.y) / cell_))};
    }
    std::size_t idx(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(nx_) + static_cast<std::size_t>(x); }

    glm::vec2 origin_;
    float cell_;
    int nx_ = 1, ny_ = 1;
    std::vector<std::vector<glm::vec2>> cells_;
};

struct UvSample {
    glm::vec2 uv;
    float speed;
};

// RK4 in the plane, fixed arc-length step, stepping either along the
// projected velocity (forward) or against it (backward). Stops at the plane
// bounds, a solid cell, stagnation, max arc length, or on approaching another
// already-placed streamline closer than `dtest`.
std::vector<UvSample> traceHalf(const core::FlowField& f, SlicePass::Axis axis, float coord, int ia, int ib, glm::vec2 start,
                                 float stepLen, float maxLen, float minSpeed, bool forward, const PlaneOccupancy& grid, float dtest)
{
    std::vector<UvSample> pts;
    auto velAt = [&](glm::vec2 q) {
        const glm::vec3 v = f.sampleVelocity(makePoint(axis, coord, q.x, q.y));
        const glm::vec2 pv(v[ia], v[ib]);
        return forward ? pv : -pv;
    };

    glm::vec2 uv = start;
    glm::vec2 u = velAt(uv);
    float speed = glm::length(u);
    if (speed < minSpeed)
        return pts;

    float len = 0.0f;
    while (len < maxLen) {
        const float s1 = glm::length(u);
        if (s1 < minSpeed)
            break;
        const float dt = stepLen / s1;
        const glm::vec2 k1 = u;
        const glm::vec2 k2 = velAt(uv + 0.5f * dt * k1);
        const glm::vec2 k3 = velAt(uv + 0.5f * dt * k2);
        const glm::vec2 k4 = velAt(uv + dt * k3);
        const glm::vec2 next = uv + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);

        const glm::vec3 p3 = makePoint(axis, coord, next.x, next.y);
        if (!f.contains(p3) || f.isSolid(p3))
            break;
        if (grid.within(next, dtest))
            break;

        len += glm::length(next - uv);
        uv = next;
        u = velAt(uv);
        speed = glm::length(u);
        pts.push_back({uv, speed});
        if (speed < minSpeed)
            break;
    }
    return pts;
}

// Traces both directions from `seed` and splices them into one ordered line
// (backward reversed, seed, forward). Returns false for degenerate lines
// (fewer than 2 total points).
bool traceStreamline(const core::FlowField& f, SlicePass::Axis axis, float coord, int ia, int ib, glm::vec2 seed, float stepLen,
                      float maxLen, float minSpeed, const PlaneOccupancy& grid, float dtest, std::vector<glm::vec2>& uvOut,
                      std::vector<float>& speedOut)
{
    const glm::vec3 seed3 = makePoint(axis, coord, seed.x, seed.y);
    const glm::vec3 v0 = f.sampleVelocity(seed3);
    const glm::vec2 v0p(v0[ia], v0[ib]);
    const float s0 = glm::length(v0p);

    const std::vector<UvSample> fwd = traceHalf(f, axis, coord, ia, ib, seed, stepLen, maxLen, minSpeed, true, grid, dtest);
    const std::vector<UvSample> bwd = traceHalf(f, axis, coord, ia, ib, seed, stepLen, maxLen, minSpeed, false, grid, dtest);

    uvOut.clear();
    speedOut.clear();
    uvOut.reserve(bwd.size() + 1 + fwd.size());
    speedOut.reserve(uvOut.capacity());
    for (auto it = bwd.rbegin(); it != bwd.rend(); ++it) {
        uvOut.push_back(it->uv);
        speedOut.push_back(it->speed);
    }
    uvOut.push_back(seed);
    speedOut.push_back(s0);
    for (const auto& s : fwd) {
        uvOut.push_back(s.uv);
        speedOut.push_back(s.speed);
    }
    return uvOut.size() >= 2;
}

} // namespace

SlicePass::SlicePass()
    : planeShader_(gl::Shader::fromFiles("slice.vert", "slice.frag")), lineShader_(gl::Shader::fromFiles("lines.vert", "lines.frag")),
      streamShader_(gl::Shader::fromFiles("streamline.vert", "streamline.geom", "streamline.frag"))
{
    colormap_.set(static_cast<ColormapKind>(colormapIndex_));
    resetRangeForQuantity();

    streamVao_ = gl::createVertexArray();
    streamVbo_ = gl::createBuffer();
    glVertexArrayVertexBuffer(streamVao_.id(), 0, streamVbo_.id(), 0, sizeof(PlaneStreamPoint));
    glEnableVertexArrayAttrib(streamVao_.id(), 0);
    glVertexArrayAttribFormat(streamVao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(PlaneStreamPoint, pos));
    glVertexArrayAttribBinding(streamVao_.id(), 0, 0);
    glEnableVertexArrayAttrib(streamVao_.id(), 1);
    glVertexArrayAttribFormat(streamVao_.id(), 1, 1, GL_FLOAT, GL_FALSE, offsetof(PlaneStreamPoint, speed));
    glVertexArrayAttribBinding(streamVao_.id(), 1, 0);
    glEnableVertexArrayAttrib(streamVao_.id(), 2);
    glVertexArrayAttribFormat(streamVao_.id(), 2, 1, GL_FLOAT, GL_FALSE, offsetof(PlaneStreamPoint, arc));
    glVertexArrayAttribBinding(streamVao_.id(), 2, 0);

    arrowVao_ = gl::createVertexArray();
    arrowVbo_ = gl::createBuffer();
    glVertexArrayVertexBuffer(arrowVao_.id(), 0, arrowVbo_.id(), 0, sizeof(LineVertex));
    glEnableVertexArrayAttrib(arrowVao_.id(), 0);
    glVertexArrayAttribFormat(arrowVao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(LineVertex, pos));
    glVertexArrayAttribBinding(arrowVao_.id(), 0, 0);
    glEnableVertexArrayAttrib(arrowVao_.id(), 1);
    glVertexArrayAttribFormat(arrowVao_.id(), 1, 4, GL_FLOAT, GL_FALSE, offsetof(LineVertex, colour));
    glVertexArrayAttribBinding(arrowVao_.id(), 1, 0);
}

SlicePass::~SlicePass() = default;

void SlicePass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    updateDefaultPosition();
    rebuildPlane();
    rebuildGlyphs();
    resetStreamlineDefaults();
    retraceStreamlines();
}

void SlicePass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    updateDefaultPosition();
    resetRangeForQuantity();
    rebuildPlane();
    rebuildGlyphs();
    resetStreamlineDefaults();
    retraceStreamlines();
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

void SlicePass::resetStreamlineDefaults()
{
    if (userSetSeparation_ || !scene_.field || scene_.field->empty())
        return;
    const core::FlowField& f = *scene_.field;
    const float cell = (f.spacing.x + f.spacing.y + f.spacing.z) / 3.0f;
    streamSeparation_ = std::max(cell * 2.5f, 1e-4f);
}

void SlicePass::retraceStreamlines()
{
    const auto t0 = std::chrono::steady_clock::now();

    planeLines_.clear();
    streamLineCount_ = 0;

    if (showStreamlines_ && scene_.field && !scene_.field->empty()) {
        const core::FlowField& f = *scene_.field;
        const core::Bounds b = f.bounds();
        const int axisIdx = static_cast<int>(axis_);
        const float coord = b.min[axisIdx] + position_ * b.size()[axisIdx];
        const auto [ia, ib] = planeAxes(axis_);

        const glm::vec2 uvMin(b.min[ia], b.min[ib]);
        const glm::vec2 uvMax(b.max[ia], b.max[ib]);
        const glm::vec2 rectSize = glm::max(uvMax - uvMin, glm::vec2(1e-4f));

        const float cell = (f.spacing.x + f.spacing.y + f.spacing.z) / 3.0f;
        const float dsep = std::max(streamSeparation_, 2.0f * cell);
        const float dtest = 0.5f * dsep;
        const float stepLen = std::max(0.5f * cell, 1e-6f);
        const float maxLen = 1.5f * glm::length(rectSize);
        const float minSpeed = 1e-3f * std::max(f.freestreamSpeed, 1e-6f);

        PlaneOccupancy grid(uvMin, rectSize, dsep);

        // Seed the growth with a coarse uniform grid across the rectangle so
        // disconnected pockets of fluid (e.g. either side of a body in an
        // X-plane behind it) still get covered, not just whatever a single
        // seed's growth happens to reach.
        std::deque<glm::vec2> queue;
        constexpr int kSeedGridX = 6, kSeedGridY = 5;
        for (int gy = 0; gy < kSeedGridY; ++gy)
            for (int gx = 0; gx < kSeedGridX; ++gx) {
                const glm::vec2 t((gx + 0.5f) / kSeedGridX, (gy + 0.5f) / kSeedGridY);
                queue.push_back(uvMin + t * rectSize);
            }

        constexpr int kMaxLines = 400;
        constexpr int kMaxIterations = 4000;
        int iterations = 0;
        std::vector<glm::vec2> uv;
        std::vector<float> speed;

        while (!queue.empty() && static_cast<int>(planeLines_.size()) < kMaxLines && iterations < kMaxIterations) {
            ++iterations;
            const glm::vec2 seed = queue.front();
            queue.pop_front();
            if (grid.within(seed, dsep * 0.9f))
                continue;

            const glm::vec3 seed3 = makePoint(axis_, coord, seed.x, seed.y);
            if (!f.contains(seed3) || f.isSolid(seed3))
                continue;

            if (!traceStreamline(f, axis_, coord, ia, ib, seed, stepLen, maxLen, minSpeed, grid, dtest, uv, speed))
                continue;

            // Insert this line before generating candidates, so new seeds
            // stay `dsep` away from it too.
            for (const glm::vec2& p : uv)
                grid.insert(p);

            std::vector<PlaneStreamPoint> line;
            line.reserve(uv.size());
            float arc = 0.0f;
            line.push_back({makePoint(axis_, coord, uv[0].x, uv[0].y), speed[0], 0.0f});
            for (std::size_t i = 1; i < uv.size(); ++i) {
                arc += glm::length(uv[i] - uv[i - 1]);
                line.push_back({makePoint(axis_, coord, uv[i].x, uv[i].y), speed[i], arc});
            }

            // Candidate seeds: walk the line every `dsep` of arc length and
            // offer points `dsep` to either side, perpendicular to the local
            // flow direction.
            float nextMark = dsep;
            for (std::size_t i = 1; i < uv.size(); ++i) {
                if (line[i].arc < nextMark)
                    continue;
                const glm::vec2 d = uv[i] - uv[i - 1];
                const float dlen = glm::length(d);
                if (dlen < 1e-9f)
                    continue;
                const glm::vec2 dir = d / dlen;
                const glm::vec2 n(-dir.y, dir.x);
                for (float sgn : {1.0f, -1.0f}) {
                    const glm::vec2 cand = uv[i] + n * dsep * sgn;
                    const glm::vec3 cand3 = makePoint(axis_, coord, cand.x, cand.y);
                    if (f.contains(cand3) && !f.isSolid(cand3) && !grid.within(cand, dsep * 0.9f))
                        queue.push_back(cand);
                }
                nextMark += dsep;
            }

            ++streamLineCount_;
            planeLines_.push_back(std::move(line));
        }
    }

    uploadStreamlineGeometry();
    buildArrowheads();

    const auto t1 = std::chrono::steady_clock::now();
    streamTraceTimeMs_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
}

void SlicePass::uploadStreamlineGeometry()
{
    std::size_t total = 0;
    for (const auto& line : planeLines_)
        if (line.size() >= 2)
            total += (line.size() - 1) * 2;

    std::vector<PlaneStreamPoint> verts;
    verts.reserve(total);
    for (const auto& line : planeLines_) {
        for (std::size_t i = 0; i + 1 < line.size(); ++i) {
            verts.push_back(line[i]);
            verts.push_back(line[i + 1]);
        }
    }

    streamVertexCount_ = static_cast<int>(verts.size());
    if (streamVertexCount_ == 0) {
        glNamedBufferData(streamVbo_.id(), 0, nullptr, GL_DYNAMIC_DRAW);
        return;
    }
    glNamedBufferData(streamVbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(PlaneStreamPoint)), verts.data(), GL_DYNAMIC_DRAW);
}

void SlicePass::buildArrowheads()
{
    arrowVertexCount_ = 0;
    if (planeLines_.empty())
        return;

    const glm::vec4 colour(0.95f, 0.97f, 1.0f, 1.0f);
    const float spacing = std::max(streamSeparation_ * 1.5f, 1e-4f);
    const float armLen = std::max(0.35f * spacing, 1e-5f);
    glm::vec3 normalAxis(0.0f);
    normalAxis[static_cast<int>(axis_)] = 1.0f;

    std::vector<LineVertex> verts;
    for (const auto& line : planeLines_) {
        if (line.size() < 2)
            continue;
        float nextMark = spacing * 0.5f;
        for (std::size_t i = 1; i < line.size(); ++i) {
            if (line[i].arc < nextMark)
                continue;
            const glm::vec3 d3 = line[i].pos - line[i - 1].pos;
            const float dlen = glm::length(d3);
            if (dlen < 1e-9f)
                continue;
            const glm::vec3 dir = d3 / dlen;
            const glm::vec3 perp = glm::cross(normalAxis, dir); // in-plane, unit length
            const glm::vec3 tip = line[i].pos;
            const glm::vec3 back = tip - dir * armLen;
            const glm::vec3 barb1 = back + perp * (armLen * 0.5f);
            const glm::vec3 barb2 = back - perp * (armLen * 0.5f);
            verts.push_back({tip, colour});
            verts.push_back({barb1, colour});
            verts.push_back({tip, colour});
            verts.push_back({barb2, colour});
            nextMark += spacing;
        }
    }

    if (verts.empty())
        return;

    arrowVertexCount_ = static_cast<int>(verts.size());
    glNamedBufferData(arrowVbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(LineVertex)), verts.data(), GL_DYNAMIC_DRAW);
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

    if (glyphVertexCount_ > 0 || streamVertexCount_ > 0 || arrowVertexCount_ > 0) {
        // Everything below sits exactly on the plane; GL_LEQUAL keeps it from
        // losing the depth test against the quad drawn just before it.
        glDepthFunc(GL_LEQUAL);

        if (glyphVertexCount_ > 0) {
            lineShader_.set("uViewProj", frame.proj * frame.view);
            lineShader_.use();
            glBindVertexArray(glyphVao_.id());
            glDrawArrays(GL_LINES, 0, glyphVertexCount_);
        }

        if (streamVertexCount_ > 0) {
            const float maxSpeed = std::max(scene_.field ? scene_.field->maxSpeed() : 1.0f, 1e-3f);
            colormap_.bind(kStreamColormapUnit);
            streamShader_.set("uViewProj", frame.proj * frame.view);
            streamShader_.set("uViewport", glm::vec2(frame.viewport));
            streamShader_.set("uWidthPx", streamLineWidthPx_);
            streamShader_.set("uColorMode", streamColourBySpeed_ ? 0 : 1);
            streamShader_.set("uSolidColour", glm::vec3(0.92f, 0.94f, 0.98f));
            streamShader_.set("uSpeedMin", 0.0f);
            streamShader_.set("uSpeedMax", maxSpeed);
            streamShader_.set("uDashOn", streamlinesPlaying_ ? 1 : 0);
            streamShader_.set("uDashFreq", streamDashFreq_);
            streamShader_.set("uDashSpeed", streamDashSpeed_);
            streamShader_.set("uTime", frame.time);
            streamShader_.set("uColormap", static_cast<int>(kStreamColormapUnit));
            streamShader_.use();
            glBindVertexArray(streamVao_.id());
            glDrawArrays(GL_LINES, 0, streamVertexCount_);
        }

        if (arrowVertexCount_ > 0) {
            lineShader_.set("uViewProj", frame.proj * frame.view);
            lineShader_.use();
            glBindVertexArray(arrowVao_.id());
            glDrawArrays(GL_LINES, 0, arrowVertexCount_);
        }

        glDepthFunc(GL_LESS);
    }

    if (blending) {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

void SlicePass::drawUi()
{
    bool planeChanged = false;

    // ---- Axis (buttons) --------------------------------------------------
    ImGui::TextUnformatted("Axis");
    ImGui::SameLine();
    for (int i = 0; i < 3; ++i) {
        if (i)
            ImGui::SameLine();
        const bool active = axis_ == static_cast<Axis>(i);
        if (active)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(i == 0 ? "X" : i == 1 ? "Y" : "Z", ImVec2(28, 0))) {
            axis_ = static_cast<Axis>(i);
            userSetPosition_ = true;
            planeChanged = true;
        }
        if (active)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    ImGui::Checkbox("Cut model", &cutModel_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Hide the part of the model between you and the plane");

    // ---- Position (world units, live) ------------------------------------
    bool haveBounds = false;
    float axisMin = 0.0f, axisMax = 1.0f;
    if (scene_.field && !scene_.field->empty()) {
        const core::Bounds b = scene_.field->bounds();
        const int axisIdx = static_cast<int>(axis_);
        axisMin = b.min[axisIdx];
        axisMax = b.max[axisIdx];
        haveBounds = true;
    }
    float coord = haveBounds ? axisMin + position_ * (axisMax - axisMin) : position_;
    ImGui::SetNextItemWidth(-1);
    if (ImGui::SliderFloat("Position (m)", &coord, haveBounds ? axisMin : 0.0f, haveBounds ? axisMax : 1.0f, "%.3f m")) {
        position_ = haveBounds ? std::clamp((coord - axisMin) / std::max(axisMax - axisMin, 1e-6f), 0.0f, 1.0f) : coord;
        userSetPosition_ = true;
        planeChanged = true;
    }
    // Optional keyboard nudge (one grid cell along the slice axis), only
    // while no text field elsewhere in the UI has keyboard focus.
    if (haveBounds && scene_.field && !ImGui::GetIO().WantTextInput) {
        const int axisIdx = static_cast<int>(axis_);
        const int cells = std::max(scene_.field->dims[axisIdx] - 1, 1);
        const float step = 1.0f / static_cast<float>(cells);
        float nudge = 0.0f;
        if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket))
            nudge = -step;
        if (ImGui::IsKeyPressed(ImGuiKey_RightBracket))
            nudge = step;
        if (nudge != 0.0f) {
            position_ = std::clamp(position_ + nudge, 0.0f, 1.0f);
            userSetPosition_ = true;
            planeChanged = true;
        }
    }
    ImGui::TextDisabled("[ / ] nudges the plane one grid cell along the axis.");

    // ---- Quantity ----------------------------------------------------------
    int quantityIdx = static_cast<int>(quantity_);
    static const char* quantityNames[] = {"Speed |U|", "Pressure Cp", "Velocity Ux", "Vorticity |w|"};
    if (ImGui::Combo("Quantity", &quantityIdx, quantityNames, 4)) {
        quantity_ = static_cast<Quantity>(quantityIdx);
        resetRangeForQuantity();
    }

    // ---- In-plane streamlines: on/off + spacing (live retrace) ------------
    bool streamChanged = false;
    if (ImGui::Checkbox("In-plane streamlines", &showStreamlines_))
        streamChanged = true;
    if (showStreamlines_) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        const float cell = (scene_.field && !scene_.field->empty())
                                ? (scene_.field->spacing.x + scene_.field->spacing.y + scene_.field->spacing.z) / 3.0f
                                : 0.01f;
        if (ImGui::SliderFloat("Spacing (m)", &streamSeparation_, std::max(2.0f * cell, 1e-4f), std::max(40.0f * cell, 0.2f), "%.3f")) {
            userSetSeparation_ = true;
            streamChanged = true;
        }
    }

    if (planeChanged)
        rebuildPlane();
    if (planeChanged && showGlyphs_)
        rebuildGlyphs();
    if (planeChanged || streamChanged)
        retraceStreamlines();
    ImGui::TextDisabled("Traced in %.2f ms, %d line%s.", streamTraceTimeMs_, streamLineCount_, streamLineCount_ == 1 ? "" : "s");

    // ---- Advanced -----------------------------------------------------------
    if (ImGui::TreeNode("Advanced##slice")) {
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

        if (showStreamlines_) {
            ImGui::Separator();
            ImGui::TextUnformatted("In-plane streamlines");
            ImGui::Checkbox("Colour by speed##stream", &streamColourBySpeed_);
            ImGui::SliderFloat("Line width (px)##stream", &streamLineWidthPx_, 0.5f, 6.0f, "%.1f");
            ImGui::DragFloat("Dash frequency (1/m)", &streamDashFreq_, 0.1f, 0.1f, 50.0f, "%.2f");
            ImGui::DragFloat("Dash speed", &streamDashSpeed_, 0.1f, -20.0f, 20.0f, "%.2f");
        }

        // Colour bar.
        ImGui::Separator();
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

        ImGui::TreePop();
    }
}

bool SlicePass::cutPlane(glm::vec4& plane) const
{
    if (!enabled || !cutModel_ || !scene_.field || scene_.field->empty())
        return false;
    const core::Bounds b = scene_.field->bounds();
    const int axisIdx = static_cast<int>(axis_);
    glm::vec3 n(0.0f);
    n[axisIdx] = 1.0f;
    const float coord = b.min[axisIdx] + position_ * b.size()[axisIdx];
    plane = glm::vec4(n, -coord);
    return true;
}

} // namespace render
