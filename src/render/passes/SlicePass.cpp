#include "render/passes/SlicePass.h"

namespace render {

// TODO: stub, to be implemented.
SlicePass::SlicePass() = default;
SlicePass::~SlicePass() = default;

void SlicePass::onBodyChanged(const SceneRefs& scene) { scene_ = scene; }
void SlicePass::onFieldChanged(const SceneRefs& scene) { scene_ = scene; }
void SlicePass::update(const FrameContext&) {}
void SlicePass::draw(const FrameContext&) {}
void SlicePass::drawUi() {}

} // namespace render
