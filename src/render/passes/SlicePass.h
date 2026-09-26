#pragma once

#include "render/RenderPass.h"

namespace render {

// Axis-aligned slice plane through the flow, coloured by speed or pressure.
class SlicePass : public RenderPass {
public:
    SlicePass();
    ~SlicePass() override;

    const char* name() const override { return "Slice"; }
    void onBodyChanged(const SceneRefs& scene) override;
    void onFieldChanged(const SceneRefs& scene) override;
    void update(const FrameContext& frame) override;
    void draw(const FrameContext& frame) override;
    void drawUi() override;

private:
    SceneRefs scene_;
};

} // namespace render
