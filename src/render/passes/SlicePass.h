#pragma once

#include "render/Colormap.h"
#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

#include <glm/glm.hpp>

#include <vector>

namespace render {

// One sample of an in-plane (2-D) streamline traced on the slice: world
// position, the local in-plane speed and the cumulative arc length from the
// seed. Layout matches the {pos, speed, arc} vertex the streamline shaders
// expect (see streamline.vert), so those shaders can be reused unmodified.
struct PlaneStreamPoint {
    glm::vec3 pos{0.0f};
    float speed = 0.0f;
    float arc = 0.0f;
};

// Axis-aligned cut plane through the flow grid, coloured by a flow quantity
// (the classic CFD "contour plane"). Optional iso-contour lines, in-plane
// velocity glyphs and evenly-spaced in-plane streamlines, sampled on the CPU
// from the FlowField.
class SlicePass : public RenderPass {
public:
    enum class Axis { X = 0, Y = 1, Z = 2 };
    // Spin: the vorticity component through the plane, signed, so the two
    // directions of rotation get opposite colours (zero in the middle).
    enum class Quantity { Speed = 0, Cp, Ux, Vorticity, Spin };

    SlicePass();
    ~SlicePass() override;

    const char* name() const override { return "Slice"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

    // The toolbar's Play button: while true, the in-plane streamlines' dash
    // pattern marches along the flow direction; otherwise they sit still.
    void setPlaying(bool playing) override { streamlinesPlaying_ = playing; }

    // True while this pass is drawing its own in-plane streamlines, so the
    // app can hide the free-roaming 3-D StreamlinePass and avoid showing the
    // same flow twice.
    bool ownsStreamlines() const { return enabled && showStreamlines_; }
    // Colour the plane by `q` (as picking it in the panel does).
    void setQuantity(Quantity q);

    // World-space plane (n.xyz, d) with n.p + d = 0 on the slice, when the
    // slice is visible and set to cut the model away on the viewer's side.
    bool cutPlane(glm::vec4& plane) const;

private:
    void updateDefaultPosition();
    void rebuildPlane();
    void rebuildGlyphs();
    void computeAutoRange();
    void resetRangeForQuantity();

    void resetStreamlineDefaults();
    void retraceStreamlines();
    void uploadStreamlineGeometry();
    void buildArrowheads();

    SceneRefs scene_;

    gl::Shader planeShader_;
    gl::Shader lineShader_;
    // In-plane streamlines reuse the 3-D StreamlinePass's ribbon shaders
    // (streamline.vert/geom/frag) read-only: identical {pos, speed, arc}
    // vertex layout and identical dash-animation uniforms.
    gl::Shader streamShader_;
    ColormapTexture colormap_;

    gl::VertexArray quadVao_;
    gl::Buffer quadVbo_;
    int quadVertexCount_ = 0;

    gl::VertexArray glyphVao_;
    gl::Buffer glyphVbo_;
    int glyphVertexCount_ = 0;

    gl::VertexArray streamVao_;
    gl::Buffer streamVbo_;
    int streamVertexCount_ = 0; // 2 per segment

    gl::VertexArray arrowVao_;
    gl::Buffer arrowVbo_;
    int arrowVertexCount_ = 0;

    // UI state.
    Axis axis_ = Axis::Y;
    float position_ = 0.5f; // fraction across the grid bounds on `axis_`
    bool cutModel_ = false; // clip the body at the plane (CAD-style section view); off: whole model, air-only plane
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

    // In-plane streamlines (evenly spaced, Jobard & Lefer 1997).
    bool showStreamlines_ = true;
    float streamSeparation_ = 0.1f; // world units: target spacing between lines
    bool userSetSeparation_ = false;
    bool streamColourBySpeed_ = false; // false = white/near-white
    float streamLineWidthPx_ = 1.6f;
    float streamDashFreq_ = 6.0f;  // stripes per metre of arc length
    float streamDashSpeed_ = 3.0f; // stripe travel speed while playing
    bool streamlinesPlaying_ = false;
    int streamLineCount_ = 0;
    float streamTraceTimeMs_ = 0.0f;

    std::vector<std::vector<PlaneStreamPoint>> planeLines_;
};

} // namespace render
