#pragma once

#include "render/RenderPass.h"

namespace render {

// GPU particles advected through the velocity field by a compute shader.
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
    SceneRefs scene_;
};

} // namespace render
