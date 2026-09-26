#include "render/passes/StreamlinePass.h"

namespace render {

// TODO: stub, to be implemented.
StreamlinePass::StreamlinePass() = default;
StreamlinePass::~StreamlinePass() = default;

void StreamlinePass::onBodyChanged(const SceneRefs& scene) { scene_ = scene; }
void StreamlinePass::onFieldChanged(const SceneRefs& scene) { scene_ = scene; }
void StreamlinePass::update(const FrameContext&) {}
void StreamlinePass::draw(const FrameContext&) {}
void StreamlinePass::drawUi() {}

} // namespace render
