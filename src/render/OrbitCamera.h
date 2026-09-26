#pragma once

#include "core/SurfaceMesh.h"

#include <glm/glm.hpp>

namespace render {

// Camera orbiting a target point. World is +z up; yaw is measured in the xy
// plane from +x, pitch is elevation above the xy plane (radians).
class OrbitCamera {
public:
    glm::vec3 target{0.0f};
    float distance = 5.0f;
    float yaw = glm::radians(-135.0f);
    float pitch = glm::radians(25.0f);
    float fovY = glm::radians(40.0f);

    glm::vec3 eye() const;
    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;

    void orbit(float dYaw, float dPitch); // cancels any in-flight flyTo()
    void pan(const glm::vec2& pixels, float viewportHeight); // screen-space drag
    void zoom(float factor);                                 // distance *= factor

    // Centre on the box and pick a distance that shows it whole.
    void frame(const core::Bounds& box);

    // Smoothly animate yaw/pitch to the given orientation (radians), easing
    // in and out over `seconds`. Yaw takes the shortest path. A subsequent
    // orbit() call cancels the animation; update() must be called each frame
    // (with the frame's delta time) to advance it.
    void flyTo(float targetYaw, float targetPitch, float seconds = 0.35f);
    void update(float dt);
    bool animating() const { return animating_; }

private:
    float sceneRadius_ = 1.0f; // used for near/far planes

    bool animating_ = false;
    float animElapsed_ = 0.0f;
    float animDuration_ = 0.35f;
    float animStartYaw_ = 0.0f;
    float animStartPitch_ = 0.0f;
    float animTargetYaw_ = 0.0f;
    float animTargetPitch_ = 0.0f;
};

} // namespace render
