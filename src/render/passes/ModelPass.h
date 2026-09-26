#pragma once

#include "render/RenderPass.h"

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
    SceneRefs scene_;
};

} // namespace render
