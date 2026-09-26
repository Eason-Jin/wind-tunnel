#pragma once

#include "render/RenderPass.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

namespace render {

// Tunnel outline: wireframe box of the flow grid plus a floor grid and a
// flow-direction arrow at the inlet. Gives spatial context to everything else.
class DomainPass : public RenderPass {
public:
    DomainPass();

    const char* name() const override { return "Tunnel"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    void rebuild();

    SceneRefs scene_;
    gl::Shader shader_;
    gl::VertexArray vao_;
    gl::Buffer vbo_;
    int vertexCount_ = 0;
    bool showGrid_ = true;
};

} // namespace render
