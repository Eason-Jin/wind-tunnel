#pragma once

#include "render/Colormap.h"
#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

#include <glm/glm.hpp>

#include <vector>

namespace render {

// One sample along a traced streamline (world position, local speed and the
// cumulative arc length from the seed, used for colouring and the dash
// animation).
struct StreamlinePoint {
    glm::vec3 pos{0.0f};
    float speed = 0.0f;
    float arc = 0.0f;
};

// Streamlines traced through the velocity field from a rectangular seed rake
// (RK4, CPU, parallelised across seeds). Rendered as screen-space-width
// ribbons produced by a geometry shader so they read as tubes at any zoom
// level (core profile caps glLineWidth at 1px).
class StreamlinePass : public RenderPass {
public:
    StreamlinePass();
    ~StreamlinePass() override;

    const char* name() const override { return "Streamline"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    enum class RakeMode { Rectangular = 0, SingleLine = 1 };

    void computeDefaults();
    std::vector<glm::vec3> buildSeeds() const;
    void retrace();
    void uploadGeometry();

    SceneRefs scene_;

    // Rake placement, in world metres. The rake is the rectangle
    // [centreY_-width_/2, centreY_+width_/2] x [centreZ_-height_/2, centreZ_+height_/2]
    // at x = rakeX_; SingleLine mode ignores width_/seedsY_ and seeds one
    // vertical line at centreY_.
    RakeMode mode_ = RakeMode::Rectangular;
    float rakeX_ = 0.0f;
    float centreY_ = 0.0f;
    float centreZ_ = 0.0f;
    float width_ = 1.0f;
    float height_ = 1.0f;
    int seedsY_ = 16;
    int seedsZ_ = 10;
    bool haveDefaults_ = false;

    // Trace parameters.
    float stepCells_ = 0.4f; // RK4 spatial step, in units of the local cell size
    int maxSteps_ = 3000;

    // Appearance.
    float lineWidthPx_ = 2.5f;
    int colorMode_ = 0; // 0 = speed (colormap), 1 = solid colour
    ColormapTexture colormap_{ColormapKind::Turbo};
    float speedMin_ = 0.0f;
    float speedMax_ = 1.0f;
    glm::vec3 solidColour_{0.85f, 0.9f, 1.0f};
    bool dashOn_ = false;
    float dashFreq_ = 4.0f;  // stripes per metre of arc length
    float dashSpeed_ = 2.0f; // stripe travel speed, metres/second equivalent

    bool dirty_ = true; // UI param changed since the last retrace; consumed in update()

    // Traced geometry (CPU) and stats.
    std::vector<std::vector<StreamlinePoint>> lines_;
    int lineCount_ = 0;
    float traceTimeMs_ = 0.0f;

    // GPU: vertex+geometry+fragment program, plus one VAO/VBO holding every
    // traced segment as an independent GL_LINES pair.
    gl::Shader shader_;
    gl::VertexArray vao_;
    gl::Buffer vbo_;
    int vertexCount_ = 0; // 2 per segment
};

} // namespace render
