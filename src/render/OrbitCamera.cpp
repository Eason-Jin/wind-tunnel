#include "render/OrbitCamera.h"

#include <glm/gtc/constants.hpp>
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
    animating_ = false;
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

void OrbitCamera::flyTo(float targetYaw, float targetPitch, float seconds)
{
    // Shortest-path delta, wrapped into [-pi, pi], applied to the current yaw
    // so the animation never spins the long way around.
    float delta = std::fmod(targetYaw - yaw, glm::two_pi<float>());
    if (delta > glm::pi<float>())
        delta -= glm::two_pi<float>();
    else if (delta < -glm::pi<float>())
        delta += glm::two_pi<float>();

    animStartYaw_ = yaw;
    animStartPitch_ = pitch;
    animTargetYaw_ = yaw + delta;
    animTargetPitch_ = std::clamp(targetPitch, glm::radians(-89.0f), glm::radians(89.0f));
    animElapsed_ = 0.0f;
    animDuration_ = std::max(seconds, 1e-4f);
    animating_ = true;
}

void OrbitCamera::update(float dt)
{
    if (!animating_)
        return;
    animElapsed_ += std::max(dt, 0.0f);
    float t = std::clamp(animElapsed_ / animDuration_, 0.0f, 1.0f);
    // Ease-in-out (smoothstep-like cubic).
    const float eased = t * t * (3.0f - 2.0f * t);
    yaw = animStartYaw_ + (animTargetYaw_ - animStartYaw_) * eased;
    pitch = animStartPitch_ + (animTargetPitch_ - animStartPitch_) * eased;
    if (t >= 1.0f) {
        yaw = animTargetYaw_;
        pitch = animTargetPitch_;
        animating_ = false;
    }
}

} // namespace render
