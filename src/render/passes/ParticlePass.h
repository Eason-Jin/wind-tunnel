#pragma once

#include "render/Colormap.h"
#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

#include <glm/glm.hpp>

#include <cstdint>

namespace render {

// GPU particles advected through the velocity field by a compute shader
// (RK2), rendered as soft point sprites or motion streaks. State (position,
// previous position, age, lifetime) lives entirely in an SSBO; the vertex
// shaders read it directly via gl_VertexID (no VBO upload).
class ParticlePass : public RenderPass {
public:
    ParticlePass();
    ~ParticlePass() override;

    const char* name() const override { return "Particle"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    void reallocate(int count);
    void computeInletBox();

    SceneRefs scene_;

    gl::Shader computeShader_;
    gl::Shader pointShader_;
    gl::Shader streakShader_;
    ColormapTexture colormap_;

    gl::Buffer particleBuffer_;
    gl::VertexArray emptyVao_;
    int particleCount_ = 0;
    int pendingCount_ = 200000;
    bool resetPending_ = true;

    // Simulation controls (UI).
    float timeScale_ = 0.1f;
    float lifetimeMin_ = 5.0f;
    float lifetimeMax_ = 10.0f;
    int emitterMode_ = 0; // 0 = inlet plane, 1 = whole domain
    bool paused_ = false;

    // Rendering controls (UI).
    bool useStreaks_ = true;
    float pointSize_ = 3.0f;
    int colorMode_ = 0; // 0 = speed colormap, 1 = smoke colour
    int colormapKind_ = 0;
    glm::vec3 smokeColor_{0.9f, 0.92f, 0.95f};
    float opacity_ = 0.35f;

    // Inlet emitter rectangle, upstream of the body, world space.
    float inletX_ = 0.0f;
    glm::vec2 inletCenter_{0.0f};
    glm::vec2 inletHalfSize_{1.0f};

    std::uint32_t frameCounter_ = 0;
    std::uint32_t seed_ = 1234567u;

    // Double-buffered GL_TIME_ELAPSED query for the compute dispatch.
    GLuint timerQueries_[2] = {0, 0};
    float computeTimeMs_ = 0.0f;
};

} // namespace render
