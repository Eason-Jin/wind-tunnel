#pragma once

#include "render/Colormap.h"
#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

#include <vector>

namespace render {

// Lit body mesh, optionally coloured by surface pressure sampled from the field.
class ModelPass : public RenderPass {
public:
    ModelPass();
    ~ModelPass() override;

    const char* name() const override { return "Model"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    enum class ColorMode { Clay = 0, SurfaceCp = 1 };

    // Texture unit for the colormap. Flow textures occupy 0/1; passes should
    // stay at >= 4.
    static constexpr GLuint kColormapUnit = 4;

    void rebuildGeometry();  // (re)uploads positions/normals, then scalar
    void recomputeScalar();  // recomputes cp_ from scene_.body / scene_.field
    void uploadScalar();     // (re)uploads cp_ to the GPU and rebinds the VAO
    void computeAutoRange();

    SceneRefs scene_{};

    gl::Shader shader_;     // model.vert + model.frag
    gl::Shader wireShader_; // model.vert + model_wire.frag
    gl::VertexArray vao_;
    gl::Buffer posNormVbo_;
    gl::Buffer scalarVbo_;
    int vertexCount_ = 0;

    std::vector<float> cp_; // per-vertex pressure coefficient, parallels body positions

    ColormapTexture colormap_;
    int colormapIndex_ = 0;

    // Defaults to Surface Cp so a freshly loaded field is visible immediately;
    // draw() falls back to Clay whenever no field is loaded.
    ColorMode colorMode_ = ColorMode::SurfaceCp;
    glm::vec3 clayColor_{0.72f, 0.75f, 0.80f};
    float cpMin_ = -1.5f;
    float cpMax_ = 1.0f;
    bool wireframe_ = false;
    float opacity_ = 1.0f;
};

} // namespace render
