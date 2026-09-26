#pragma once

#include "render/RenderPass.h"

namespace render {

// Streamlines traced through the velocity field from a seed rake (RK4).
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
    SceneRefs scene_;
};

} // namespace render
