#include "render/passes/ParticlePass.h"

namespace render {

// TODO: stub, to be implemented.
ParticlePass::ParticlePass() = default;
ParticlePass::~ParticlePass() = default;

void ParticlePass::onBodyChanged(const SceneRefs& scene) { scene_ = scene; }
void ParticlePass::onFieldChanged(const SceneRefs& scene) { scene_ = scene; }
void ParticlePass::update(const FrameContext&) {}
void ParticlePass::draw(const FrameContext&) {}
void ParticlePass::drawUi() {}

} // namespace render
