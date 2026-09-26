#pragma once

#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

#include <cstdint>
#include <vector>

namespace render {

// Isosurface of the (normalised) Q-criterion, showing coherent vortical
// structures (e.g. trailing vortex pairs) that a surface-pressure or
// streamline view doesn't make legible. Q is computed on the CPU from the
// velocity gradient whenever the field changes; the isosurface itself is
// re-meshed with marching cubes whenever the threshold / near-wall cutoff /
// colour mode change (debounced while a slider is being dragged).
class VortexPass : public RenderPass {
public:
    VortexPass();
    ~VortexPass() override;

    const char* name() const override { return "Vortices"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    enum class ColorMode { SwirlDirection = 0, VelocityMagnitude = 1 };

    struct Vertex {
        glm::vec3 pos{0.0f};
        glm::vec3 normal{0.0f};
        glm::vec3 color{1.0f};
    };

    static constexpr std::uint8_t kMaxWallDist = 8; // BFS cap; also caps the "hide near-wall" slider

    // --- CPU field analysis (recomputed on field change) --------------------
    void computeQCriterion();     // fills qStar_, gradQ_, omegaXNorm_, speedNorm_, wallDist_
    void remesh();                // marching cubes over qStar_ at threshold_, uploads geometry
    void uploadGeometry(const std::vector<Vertex>& verts);
    void markMeshDirty() { meshDirty_ = true; }

    SceneRefs scene_{};

    // Per-cell CPU analysis, parallel arrays sized field_->cellCount().
    std::vector<float> qStar_;           // normalised Q; very negative where invalid (solid / near-wall)
    std::vector<glm::vec3> gradQ_;       // gradient of qStar_, used for vertex normals
    std::vector<float> omegaXNorm_;      // streamwise vorticity, normalised by U_inf/L
    std::vector<float> speedNorm_;       // |velocity| / freestream speed
    std::vector<std::uint8_t> wallDist_; // cells to nearest solid cell, capped at kMaxWallDist
    glm::ivec3 dims_{0};
    glm::vec3 origin_{0.0f};
    glm::vec3 spacing_{1.0f};
    float qScale_ = 1.0f; // (U_inf / L)^2 used to normalise Q
    bool haveField_ = false;
    float computeTimeMs_ = 0.0f; // Q-criterion compute time

    // UI parameters.
    float thresholdLog_ = 0.0f; // slider is on a log10 scale; threshold_ = 10^thresholdLog_
    float threshold_ = 1.0f;
    ColorMode colorMode_ = ColorMode::SwirlDirection;
    float opacity_ = 0.6f;
    int hideNearWall_ = 2; // cells from solid to exclude from the isosurface

    // Debounce/throttle: remesh on release, or at most ~5 times a second while dragging.
    bool meshDirty_ = false;
    float lastRemeshTime_ = -1e6f;
    static constexpr float kDebounceSeconds = 0.2f; // ~5 Hz cap while dragging

    // Mesh stats.
    int triangleCount_ = 0;
    float meshTimeMs_ = 0.0f;

    // GL resources.
    gl::Shader shader_;
    gl::VertexArray vao_;
    gl::Buffer vbo_;
    int vertexCount_ = 0;
};

} // namespace render
