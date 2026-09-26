#pragma once

#include "core/FlowField.h"
#include "core/SurfaceMesh.h"
#include "render/FlowTextures.h"

#include <glm/glm.hpp>

namespace render {

// Per-frame data handed to every pass.
struct FrameContext {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};
    glm::vec3 eye{0.0f};
    glm::ivec2 viewport{1};
    float time = 0.0f; // seconds since start
    float dt = 0.0f;   // seconds since last frame (clamped, > 0)
};

// Scene state shared by all passes. Pointers stay valid until the next
// on*Changed call; nullptr / empty means "nothing loaded".
struct SceneRefs {
    const core::SurfaceMesh* body = nullptr; // in metres, world frame (+x flow, +z up)
    const core::FlowField* field = nullptr;
    const FlowTextures* flowTextures = nullptr;
};

// One visual layer. The app owns a list of passes and, each frame, calls
// update() on all enabled passes, then draw() in list order with depth testing
// enabled and the default (or offscreen) framebuffer bound.
//
// Rules for implementations:
//  - Construct GL resources lazily or in the constructor (a context exists).
//  - Leave GL state as found: depth test ON, depth write ON, blending OFF,
//    culling OFF, program/VAO bindings may be left dirty.
//  - drawUi() is called inside an open ImGui window; use ImGui widgets only
//    (no Begin/End of new windows).
class RenderPass {
public:
    virtual ~RenderPass() = default;

    virtual const char* name() const = 0;

    virtual void onBodyChanged(const SceneRefs&) {}
    virtual void onFieldChanged(const SceneRefs&) {}

    virtual void update(const FrameContext&) {}
    virtual void draw(const FrameContext&) = 0;
    virtual void drawUi() {}

    bool enabled = true;
};

} // namespace render
