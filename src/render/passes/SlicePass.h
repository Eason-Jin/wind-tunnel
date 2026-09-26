#pragma once

#include "render/Colormap.h"
#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

namespace render {

// Axis-aligned cut plane through the flow grid, coloured by a flow quantity
// (the classic CFD "contour plane"). Optional iso-contour lines and in-plane
// velocity glyphs, sampled on the CPU from the FlowField.
class SlicePass : public RenderPass {
public:
    enum class Axis { X = 0, Y = 1, Z = 2 };
    enum class Quantity { Speed = 0, Cp, Ux, Vorticity };

    SlicePass();
    ~SlicePass() override;

    const char* name() const override { return "Slice"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    void updateDefaultPosition();
    void rebuildPlane();
    void rebuildGlyphs();
    void computeAutoRange();
    void resetRangeForQuantity();

    SceneRefs scene_;

    gl::Shader planeShader_;
    gl::Shader lineShader_;
    ColormapTexture colormap_;

    gl::VertexArray quadVao_;
    gl::Buffer quadVbo_;
    int quadVertexCount_ = 0;

    gl::VertexArray glyphVao_;
    gl::Buffer glyphVbo_;
    int glyphVertexCount_ = 0;

    // UI state.
    Axis axis_ = Axis::Y;
    float position_ = 0.5f; // fraction across the grid bounds on `axis_`
    bool userSetPosition_ = false;
    Quantity quantity_ = Quantity::Speed;
    int colormapIndex_ = 0;
    float rangeMin_ = 0.0f;
    float rangeMax_ = 1.0f;
    bool showSolid_ = true; // false = discard solid fragments
    bool showContours_ = false;
    int contourCount_ = 10;
    bool showGlyphs_ = false;
    bool glyphColourBySpeed_ = false; // false = white (visible over any colormap)
    int glyphDensity_ = 14; // samples along the longer in-plane axis
    float opacity_ = 1.0f;
};

} // namespace render
