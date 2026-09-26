#include "render/passes/ParticlePass.h"

#include <imgui.h>

#include <algorithm>
#include <cstdint>

namespace render {

namespace {

// Matches the `Particle` struct in particles.comp / particles_points.vert /
// particles_streak.vert (std430: two vec4s, 32 bytes, no padding).
struct GpuParticle {
    glm::vec4 posAge;   // xyz = world position, w = age (s)
    glm::vec4 prevLife; // xyz = previous position, w = lifetime (s)
};

constexpr GLuint kParticleBinding = 0; // SSBO binding, matches shaders
constexpr GLuint kColormapUnit = 4;    // texture unit, matches the ">= 4" convention
constexpr int kLocalSize = 256;
constexpr int kMinParticles = 10000;
constexpr int kMaxParticles = 1000000;

} // namespace

ParticlePass::ParticlePass()
    : computeShader_(gl::Shader::computeFromFile("particles.comp")),
      pointShader_(gl::Shader::fromFiles("particles_points.vert", "particles_points.frag")),
      streakShader_(gl::Shader::fromFiles("particles_streak.vert", "particles_streak.frag")),
      colormap_(ColormapKind::Turbo)
{
    emptyVao_ = gl::createVertexArray();
    glCreateQueries(GL_TIME_ELAPSED, 2, timerQueries_);
    reallocate(pendingCount_);
}

ParticlePass::~ParticlePass()
{
    if (timerQueries_[0] || timerQueries_[1])
        glDeleteQueries(2, timerQueries_);
}

void ParticlePass::reallocate(int count)
{
    count = std::clamp(count, kMinParticles, kMaxParticles);
    particleCount_ = count;
    pendingCount_ = count;
    particleBuffer_ = gl::createBuffer();
    glNamedBufferStorage(particleBuffer_.id(), static_cast<GLsizeiptr>(count) * sizeof(GpuParticle), nullptr, 0);
    resetPending_ = true;
}

void ParticlePass::computeInletBox()
{
    core::Bounds domain;
    bool haveDomain = false;
    if (scene_.field && !scene_.field->empty()) {
        domain = scene_.field->bounds();
        haveDomain = true;
    }

    if (scene_.body && !scene_.body->empty()) {
        const core::Bounds b = scene_.body->bounds();
        const glm::vec3 c = b.centre();
        const glm::vec3 sz = b.size();
        inletCenter_ = glm::vec2(c.y, c.z);
        // Inlet rectangle covers 1.5x the body's cross-section (half-size = 0.75x).
        inletHalfSize_ = 0.75f * glm::vec2(std::max(sz.y, 1e-3f), std::max(sz.z, 1e-3f));
        inletX_ = haveDomain ? glm::mix(domain.min.x, b.min.x, 0.4f) : (b.min.x - std::max(sz.x, 1.0f));
    } else if (haveDomain) {
        const glm::vec3 c = domain.centre();
        inletCenter_ = glm::vec2(c.y, c.z);
        inletHalfSize_ = 0.5f * glm::vec2(domain.size().y, domain.size().z);
        inletX_ = glm::mix(domain.min.x, domain.max.x, 0.1f);
    }
}

void ParticlePass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    computeInletBox();
    resetPending_ = true;
}

void ParticlePass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    computeInletBox();
    if (scene_.field && !scene_.field->empty()) {
        // Default so particles cross the domain in a few seconds: freestream *
        // timeScale * 4s ~= domain length along the flow direction.
        const float domainLen = std::max(scene_.field->bounds().size().x, 1e-3f);
        const float freestream = std::max(scene_.field->freestreamSpeed, 1e-3f);
        timeScale_ = std::clamp(domainLen / (freestream * 4.0f), 0.01f, 2.0f);
    }
    resetPending_ = true;
}

void ParticlePass::update(const FrameContext& frame)
{
    if (!scene_.field || scene_.field->empty() || !scene_.flowTextures || !scene_.flowTextures->valid())
        return;

    if (pendingCount_ != particleCount_)
        reallocate(pendingCount_);

    scene_.flowTextures->bind();

    scene_.flowTextures->setUniforms(computeShader_);
    computeShader_.set("uDt", frame.dt);
    computeShader_.set("uTimeScale", timeScale_);
    computeShader_.set("uFrame", static_cast<int>(frameCounter_));
    computeShader_.set("uSeed", static_cast<int>(seed_));
    computeShader_.set("uParticleCount", particleCount_);
    computeShader_.set("uReset", resetPending_ ? 1 : 0);
    computeShader_.set("uPaused", paused_ ? 1 : 0);
    computeShader_.set("uEmitterMode", emitterMode_);
    computeShader_.set("uLifetimeMin", lifetimeMin_);
    computeShader_.set("uLifetimeMax", lifetimeMax_);
    computeShader_.set("uInletX", inletX_);
    computeShader_.set("uInletCenter", inletCenter_);
    computeShader_.set("uInletHalfSize", inletHalfSize_);
    computeShader_.use();

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kParticleBinding, particleBuffer_.id());

    const int qIndex = static_cast<int>(frameCounter_ % 2u);
    glBeginQuery(GL_TIME_ELAPSED, timerQueries_[qIndex]);

    const GLuint groups = (static_cast<GLuint>(particleCount_) + kLocalSize - 1u) / kLocalSize;
    glDispatchCompute(groups, 1, 1);

    glEndQuery(GL_TIME_ELAPSED);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

    const int readIndex = 1 - qIndex;
    GLint available = 0;
    glGetQueryObjectiv(timerQueries_[readIndex], GL_QUERY_RESULT_AVAILABLE, &available);
    if (available) {
        GLuint64 ns = 0;
        glGetQueryObjectui64v(timerQueries_[readIndex], GL_QUERY_RESULT, &ns);
        computeTimeMs_ = static_cast<float>(ns) * 1e-6f;
    }

    resetPending_ = false;
    ++frameCounter_;
}

void ParticlePass::draw(const FrameContext& frame)
{
    if (!scene_.field || scene_.field->empty() || !scene_.flowTextures || !scene_.flowTextures->valid())
        return;

    scene_.flowTextures->bind();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kParticleBinding, particleBuffer_.id());
    glBindVertexArray(emptyVao_.id());

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    gl::Shader& shader = useStreaks_ ? streakShader_ : pointShader_;
    scene_.flowTextures->setUniforms(shader);
    shader.set("uViewProj", frame.proj * frame.view);
    shader.set("uOpacity", opacity_);
    shader.set("uColorMode", colorMode_);
    shader.set("uSmokeColor", smokeColor_);
    if (colorMode_ == 0) {
        colormap_.bind(kColormapUnit);
        shader.set("uColormap", static_cast<int>(kColormapUnit));
    }
    shader.use();

    if (useStreaks_) {
        glDrawArrays(GL_LINES, 0, particleCount_ * 2);
    } else {
        glEnable(GL_PROGRAM_POINT_SIZE);
        shader.set("uPointSize", pointSize_);
        glDrawArrays(GL_POINTS, 0, particleCount_);
        glDisable(GL_PROGRAM_POINT_SIZE);
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void ParticlePass::drawUi()
{
    const bool fieldValid = scene_.field && !scene_.field->empty();
    if (!fieldValid)
        ImGui::TextDisabled("No flow field");

    ImGui::SliderInt("Particle count", &pendingCount_, kMinParticles, kMaxParticles, "%d", ImGuiSliderFlags_Logarithmic);
    if (ImGui::IsItemDeactivatedAfterEdit())
        reallocate(pendingCount_);

    const char* emitterNames[] = {"Inlet plane", "Whole domain"};
    if (ImGui::Combo("Emitter", &emitterMode_, emitterNames, 2))
        resetPending_ = true;

    ImGui::DragFloat("Time scale", &timeScale_, 0.001f, 0.001f, 2.0f, "%.3f");
    ImGui::DragFloatRange2("Lifetime (s)", &lifetimeMin_, &lifetimeMax_, 0.05f, 0.1f, 30.0f, "min %.2f", "max %.2f");

    ImGui::Checkbox("Streaks (vs points)", &useStreaks_);
    if (!useStreaks_)
        ImGui::SliderFloat("Point size", &pointSize_, 1.0f, 12.0f, "%.1f");

    const char* colorNames[] = {"Speed colormap", "Smoke colour"};
    ImGui::Combo("Colour mode", &colorMode_, colorNames, 2);
    if (colorMode_ == 0) {
        int kind = colormapKind_;
        if (ImGui::Combo("Colormap", &kind, kColormapNames, IM_ARRAYSIZE(kColormapNames))) {
            colormapKind_ = kind;
            colormap_.set(static_cast<ColormapKind>(kind));
        }
    } else {
        ImGui::ColorEdit3("Smoke colour", &smokeColor_.x);
    }

    ImGui::SliderFloat("Opacity", &opacity_, 0.0f, 1.0f, "%.2f");
    ImGui::Checkbox("Paused", &paused_);
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        resetPending_ = true;

    ImGui::Text("Compute: %.3f ms (%d particles)", computeTimeMs_, particleCount_);
}

} // namespace render
