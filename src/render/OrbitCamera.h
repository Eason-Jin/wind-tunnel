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

    void orbit(float dYaw, float dPitch);
    void pan(const glm::vec2& pixels, float viewportHeight); // screen-space drag
    void zoom(float factor);                                 // distance *= factor

    // Centre on the box and pick a distance that shows it whole.
    void frame(const core::Bounds& box);

private:
    float sceneRadius_ = 1.0f; // used for near/far planes
};

} // namespace render
