#include "render/passes/ModelPass.h"

namespace render {

// TODO: stub, to be implemented.
ModelPass::ModelPass() = default;
ModelPass::~ModelPass() = default;

void ModelPass::onBodyChanged(const SceneRefs& scene) { scene_ = scene; }
void ModelPass::onFieldChanged(const SceneRefs& scene) { scene_ = scene; }
void ModelPass::update(const FrameContext&) {}
void ModelPass::draw(const FrameContext&) {}
void ModelPass::drawUi() {}

} // namespace render
