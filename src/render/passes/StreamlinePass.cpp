#include "render/passes/StreamlinePass.h"

#include <imgui.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

namespace render {

namespace {

// --- CPU tracing -------------------------------------------------------------

// `direction` -1 traces upstream (against the flow).
std::vector<StreamlinePoint> traceOne(const core::FlowField& field, const glm::vec3& seed, float stepLen, int maxSteps,
                                      float direction = 1.0f)
{
    const auto velocity = [&](const glm::vec3& at) { return direction * field.sampleVelocity(at); };
    std::vector<StreamlinePoint> pts;
    if (!field.contains(seed) || field.isSolid(seed))
        return pts;

    const float minSpeed = 1e-3f * std::max(field.freestreamSpeed, 1e-6f);

    glm::vec3 p = seed;
    glm::vec3 u = velocity(p);
    float speed = glm::length(u);
    pts.push_back({p, speed, 0.0f});
    if (speed < minSpeed)
        return pts;

    float arc = 0.0f;
    for (int step = 0; step < maxSteps; ++step) {
        const glm::vec3 k1 = u;
        const float s1 = speed;
        if (s1 < minSpeed)
            break;
        const float dt = stepLen / s1; // fixed spatial step, normalised by local speed

        const glm::vec3 k2 = velocity(p + 0.5f * dt * k1);
        const glm::vec3 k3 = velocity(p + 0.5f * dt * k2);
        const glm::vec3 k4 = velocity(p + dt * k3);
        const glm::vec3 next = p + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);

        if (!field.contains(next) || field.isSolid(next))
            break;

        arc += glm::length(next - p);
        p = next;
        u = velocity(p);
        speed = glm::length(u);
        pts.push_back({p, speed, arc});
    }
    return pts;
}

// Upstream and downstream from the seed, joined into one line running with
// the flow (arc length restarts at its upstream end).
std::vector<StreamlinePoint> traceBothWays(const core::FlowField& field, const glm::vec3& seed, float stepLen, int maxSteps)
{
    std::vector<StreamlinePoint> back = traceOne(field, seed, stepLen, maxSteps, -1.0f);
    const std::vector<StreamlinePoint> forward = traceOne(field, seed, stepLen, maxSteps);
    if (back.empty())
        return forward;
    std::reverse(back.begin(), back.end());
    back.insert(back.end(), forward.begin() + (forward.empty() ? 0 : 1), forward.end());
    float arc = 0.0f;
    for (std::size_t i = 0; i < back.size(); ++i) {
        if (i > 0)
            arc += glm::length(back[i].pos - back[i - 1].pos);
        back[i].arc = arc;
    }
    return back;
}

} // namespace

StreamlinePass::StreamlinePass(StreamlineSeeding seeding)
    : seeding_(seeding), shader_(gl::Shader::fromFiles("streamline.vert", "streamline.geom", "streamline.frag"))
{
    vao_ = gl::createVertexArray();
    vbo_ = gl::createBuffer();

    glVertexArrayVertexBuffer(vao_.id(), 0, vbo_.id(), 0, sizeof(StreamlinePoint));
    glEnableVertexArrayAttrib(vao_.id(), 0);
    glVertexArrayAttribFormat(vao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(StreamlinePoint, pos));
    glVertexArrayAttribBinding(vao_.id(), 0, 0);
    glEnableVertexArrayAttrib(vao_.id(), 1);
    glVertexArrayAttribFormat(vao_.id(), 1, 1, GL_FLOAT, GL_FALSE, offsetof(StreamlinePoint, speed));
    glVertexArrayAttribBinding(vao_.id(), 1, 0);
    glEnableVertexArrayAttrib(vao_.id(), 2);
    glVertexArrayAttribFormat(vao_.id(), 2, 1, GL_FLOAT, GL_FALSE, offsetof(StreamlinePoint, arc));
    glVertexArrayAttribBinding(vao_.id(), 2, 0);
}

StreamlinePass::~StreamlinePass() = default;

void StreamlinePass::computeDefaults()
{
    if (!scene_.body || scene_.body->empty())
        return;

    const core::Bounds bb = scene_.body->bounds();
    const float bodyLenX = bb.size().x;
    const float bodyHeight = std::max(bb.size().z, 1e-4f);
    const float floorZ = bb.min.z;

    rakeX_ = bb.min.x - 0.3f * bodyLenX;
    if (scene_.field && !scene_.field->empty()) {
        const core::Bounds fb = scene_.field->bounds();
        const glm::vec3 sp = scene_.field->spacing;
        const float margin = 0.5f * std::max({sp.x, sp.y, sp.z});
        rakeX_ = glm::clamp(rakeX_, fb.min.x + margin, fb.max.x - margin);
    }

    centreY_ = bb.centre().y;
    width_ = 1.4f * bb.size().y;

    // Relative gap only: models range from millimetres to metres.
    const float smallGap = 0.02f * bodyHeight;
    const float zLow = floorZ + smallGap;
    const float zHigh = floorZ + 1.3f * bodyHeight;
    centreZ_ = 0.5f * (zLow + zHigh);
    height_ = std::max(zHigh - zLow, 1e-4f);

    seedsY_ = 16;
    seedsZ_ = 10;
    dirty_ = true;
}

std::vector<glm::vec3> StreamlinePass::buildSeeds()
{
    std::vector<glm::vec3> seeds;
    if (seeding_ == StreamlineSeeding::VortexCores) {
        if (!scene_.field || scene_.field->empty())
            return seeds;
        const core::FlowField& field = *scene_.field;
        const float L = scene_.body && !scene_.body->empty() ? scene_.body->bounds().size().x : field.bounds().size().y;
        if (coresDirty_) {
            core::VortexCoreOptions options;
            options.charLength = L;
            options.minStrength = std::pow(10.0f, minStrengthLog_);
            options.maxCores = coreCount_;
            options.minSeparation = 0.15f * L;
            cores_ = core::findVortexCores(field, options);
            coresDirty_ = false;
        }
        const float cellSize = (field.spacing.x + field.spacing.y + field.spacing.z) / 3.0f;
        for (const core::VortexCore& c : cores_)
            for (const glm::vec3& p : core::ringAround(c, ringCells_ * cellSize, linesPerCore_))
                seeds.push_back(p);
        return seeds;
    }
    const float zLow = centreZ_ - 0.5f * height_;
    const float zHigh = centreZ_ + 0.5f * height_;

    if (mode_ == RakeMode::Rectangular) {
        const int ny = std::max(seedsY_, 1);
        const int nz = std::max(seedsZ_, 1);
        const float yLow = centreY_ - 0.5f * width_;
        seeds.reserve(static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz));
        for (int k = 0; k < nz; ++k) {
            const float tz = nz > 1 ? static_cast<float>(k) / static_cast<float>(nz - 1) : 0.5f;
            const float z = zLow + tz * (zHigh - zLow);
            for (int i = 0; i < ny; ++i) {
                const float ty = ny > 1 ? static_cast<float>(i) / static_cast<float>(ny - 1) : 0.5f;
                seeds.emplace_back(rakeX_, yLow + ty * width_, z);
            }
        }
    } else {
        const int n = std::max(seedsZ_, 1);
        seeds.reserve(static_cast<std::size_t>(n));
        for (int k = 0; k < n; ++k) {
            const float tz = n > 1 ? static_cast<float>(k) / static_cast<float>(n - 1) : 0.5f;
            seeds.emplace_back(rakeX_, centreY_, zLow + tz * (zHigh - zLow));
        }
    }
    return seeds;
}

void StreamlinePass::retrace()
{
    const auto t0 = std::chrono::steady_clock::now();

    lines_.clear();
    lineCount_ = 0;

    if (scene_.field && !scene_.field->empty()) {
        const core::FlowField& field = *scene_.field;
        const std::vector<glm::vec3> seeds = buildSeeds();
        const float cellSize = (field.spacing.x + field.spacing.y + field.spacing.z) / 3.0f;
        const float stepLen = std::max(stepCells_ * cellSize, 1e-6f);

        const auto trace = [&](const glm::vec3& seed) {
            return seeding_ == StreamlineSeeding::VortexCores ? traceBothWays(field, seed, stepLen, maxSteps_)
                                                               : traceOne(field, seed, stepLen, maxSteps_);
        };
        std::vector<std::vector<StreamlinePoint>> results(seeds.size());
        const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
        const unsigned nThreads = static_cast<unsigned>(std::min<std::size_t>(hw, seeds.size()));

        if (nThreads <= 1) {
            for (std::size_t i = 0; i < seeds.size(); ++i)
                results[i] = trace(seeds[i]);
        } else {
            std::atomic<std::size_t> next{0};
            std::vector<std::thread> pool;
            pool.reserve(nThreads);
            for (unsigned t = 0; t < nThreads; ++t) {
                pool.emplace_back([&]() {
                    std::size_t i;
                    while ((i = next.fetch_add(1)) < seeds.size())
                        results[i] = trace(seeds[i]);
                });
            }
            for (auto& th : pool)
                th.join();
        }

        lines_.reserve(results.size());
        for (auto& r : results) {
            if (r.size() >= 2) {
                ++lineCount_;
                lines_.push_back(std::move(r));
            }
        }
    }

    uploadGeometry();

    const auto t1 = std::chrono::steady_clock::now();
    traceTimeMs_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
}

void StreamlinePass::uploadGeometry()
{
    std::size_t total = 0;
    for (const auto& line : lines_)
        total += (line.size() - 1) * 2;

    std::vector<StreamlinePoint> verts;
    verts.reserve(total);
    for (const auto& line : lines_) {
        for (std::size_t i = 0; i + 1 < line.size(); ++i) {
            verts.push_back(line[i]);
            verts.push_back(line[i + 1]);
        }
    }

    vertexCount_ = static_cast<int>(verts.size());
    if (vertexCount_ == 0) {
        glNamedBufferData(vbo_.id(), 0, nullptr, GL_DYNAMIC_DRAW);
        return;
    }
    glNamedBufferData(vbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(StreamlinePoint)), verts.data(), GL_DYNAMIC_DRAW);
}

void StreamlinePass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    coresDirty_ = true;
    computeDefaults();
    haveDefaults_ = true;
    retrace();
}

void StreamlinePass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    coresDirty_ = true;
    if (scene_.field && !scene_.field->empty()) {
        speedMax_ = std::max(scene_.field->maxSpeed(), 1e-3f);
        speedMin_ = 0.0f;
    }
    if (!haveDefaults_ && scene_.body && !scene_.body->empty()) {
        computeDefaults();
        haveDefaults_ = true;
    }
    retrace();
}

void StreamlinePass::update(const FrameContext&)
{
    if (dirty_) {
        retrace();
        dirty_ = false;
    }
}

void StreamlinePass::draw(const FrameContext& frame)
{
    if (vertexCount_ == 0)
        return;

    shader_.set("uViewProj", frame.proj * frame.view);
    shader_.set("uViewport", glm::vec2(frame.viewport));
    shader_.set("uWidthPx", lineWidthPx_);
    shader_.set("uColorMode", colorMode_);
    shader_.set("uSolidColour", solidColour_);
    shader_.set("uSpeedMin", speedMin_);
    shader_.set("uSpeedMax", speedMax_);
    shader_.set("uDashOn", dashOn_ ? 1 : 0);
    shader_.set("uDashFreq", dashFreq_);
    shader_.set("uDashSpeed", dashSpeed_);
    shader_.set("uTime", frame.time);

    colormap_.bind(4);
    shader_.set("uColormap", 4);

    shader_.use();
    glBindVertexArray(vao_.id());
    glDrawArrays(GL_LINES, 0, vertexCount_);
}

void StreamlinePass::drawSwirlUi(bool& changed)
{
    ImGui::TextWrapped("Lines start just beside the strongest swirling regions and are traced both ways, so they "
                       "corkscrew around each vortex.");
    bool coresChanged = false;
    coresChanged |= ImGui::SliderInt("Vortices", &coreCount_, 1, 30);
    coresChanged |= ImGui::SliderFloat("Min strength (log10 Q*)", &minStrengthLog_, -1.0f, 3.0f, "%.1f");
    changed |= ImGui::SliderInt("Lines per vortex", &linesPerCore_, 1, 12);
    changed |= ImGui::SliderFloat("Ring radius (cells)", &ringCells_, 0.5f, 8.0f, "%.1f");
    if (coresChanged) {
        coresDirty_ = true;
        changed = true;
    }
    if (cores_.empty())
        ImGui::TextDisabled("No vortices this strong: lower the minimum strength.");
    else
        ImGui::Text("%d vortices, strongest Q* %.0f", static_cast<int>(cores_.size()), cores_.front().strength);
}

void StreamlinePass::drawUi()
{
    bool changed = false;
    if (seeding_ == StreamlineSeeding::VortexCores)
        drawSwirlUi(changed);
    else
        drawRakeUi(changed);
    drawTraceUi(changed);
}

void StreamlinePass::drawRakeUi(bool& changed)
{
    int mode = static_cast<int>(mode_);
    if (ImGui::Combo("Rake mode", &mode, "Rectangular\0Single line\0")) {
        mode_ = static_cast<RakeMode>(mode);
        changed = true;
    }
    changed |= ImGui::DragFloat("Rake X", &rakeX_, 0.02f, -1000.0f, 1000.0f, "%.3f");
    changed |= ImGui::DragFloat("Centre Y", &centreY_, 0.02f, -1000.0f, 1000.0f, "%.3f");
    changed |= ImGui::DragFloat("Centre Z", &centreZ_, 0.02f, -1000.0f, 1000.0f, "%.3f");
    if (mode_ == RakeMode::Rectangular) {
        changed |= ImGui::DragFloat("Width (y)", &width_, 0.02f, 0.01f, 1000.0f, "%.3f");
        changed |= ImGui::SliderInt("Seeds Y", &seedsY_, 1, 64);
    }
    changed |= ImGui::DragFloat("Height (z)", &height_, 0.02f, 0.01f, 1000.0f, "%.3f");
    changed |= ImGui::SliderInt(mode_ == RakeMode::Rectangular ? "Seeds Z" : "Seeds (line)", &seedsZ_, 1, 128);
    if (ImGui::Button("Reset rake to default")) {
        computeDefaults();
        changed = true;
    }
}

void StreamlinePass::drawTraceUi(bool& changed)
{
    ImGui::Separator();
    changed |= ImGui::DragFloat("Step (x cell size)", &stepCells_, 0.01f, 0.05f, 4.0f, "%.2f");
    changed |= ImGui::SliderInt("Max steps", &maxSteps_, 10, 20000);

    ImGui::Separator();
    ImGui::SliderFloat("Line width (px)", &lineWidthPx_, 0.5f, 12.0f, "%.1f");
    ImGui::Combo("Colour by", &colorMode_, "Speed\0Solid colour\0");
    if (colorMode_ == 0) {
        int kind = static_cast<int>(colormap_.kind());
        if (ImGui::Combo("Colormap", &kind, kColormapNames, IM_ARRAYSIZE(kColormapNames)))
            colormap_.set(static_cast<ColormapKind>(kind));
        ImGui::DragFloatRange2("Speed range", &speedMin_, &speedMax_, 0.1f, 0.0f, 1000.0f, "%.2f");
        if (scene_.field && !scene_.field->empty() && ImGui::Button("Auto range")) {
            speedMin_ = 0.0f;
            speedMax_ = std::max(scene_.field->maxSpeed(), 1e-3f);
        }
    } else {
        ImGui::ColorEdit3("Solid colour", &solidColour_.x);
    }

    ImGui::Separator();
    ImGui::Checkbox("Dash animation", &dashOn_);
    if (dashOn_) {
        ImGui::DragFloat("Dash frequency (1/m)", &dashFreq_, 0.1f, 0.1f, 50.0f, "%.2f");
        ImGui::DragFloat("Dash speed", &dashSpeed_, 0.1f, -20.0f, 20.0f, "%.2f");
    }

    ImGui::Separator();
    ImGui::Text("%d lines, %d vertices", lineCount_, vertexCount_ / 2);
    ImGui::Text("Trace time: %.2f ms", traceTimeMs_);

    if (changed)
        dirty_ = true;
}

} // namespace render
