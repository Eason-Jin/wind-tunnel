#include "render/OrbitCamera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace render {

glm::vec3 OrbitCamera::eye() const
{
    const float cp = std::cos(pitch);
    return target + distance * glm::vec3(cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch));
}

glm::mat4 OrbitCamera::view() const
{
    return glm::lookAt(eye(), target, glm::vec3(0.0f, 0.0f, 1.0f));
}

glm::mat4 OrbitCamera::projection(float aspect) const
{
    const float nearPlane = std::max(distance - 4.0f * sceneRadius_, distance * 0.01f);
    const float farPlane = distance + 8.0f * sceneRadius_;
    return glm::perspective(fovY, aspect, nearPlane, farPlane);
}

void OrbitCamera::orbit(float dYaw, float dPitch)
{
    yaw += dYaw;
    pitch = std::clamp(pitch + dPitch, glm::radians(-89.0f), glm::radians(89.0f));
}

void OrbitCamera::pan(const glm::vec2& pixels, float viewportHeight)
{
    const glm::vec3 forward = glm::normalize(target - eye());
    const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 0.0f, 1.0f)));
    const glm::vec3 up = glm::cross(right, forward);
    const float worldPerPixel = 2.0f * distance * std::tan(0.5f * fovY) / std::max(viewportHeight, 1.0f);
    target += (-pixels.x * right + pixels.y * up) * worldPerPixel;
}

void OrbitCamera::zoom(float factor)
{
    distance = std::clamp(distance * factor, sceneRadius_ * 0.05f, sceneRadius_ * 50.0f);
}

void OrbitCamera::frame(const core::Bounds& box)
{
    target = box.centre();
    sceneRadius_ = std::max(box.radius(), 1e-4f);
    distance = sceneRadius_ / std::sin(0.5f * fovY) * 1.1f;
}

} // namespace render
