#include "core/FlowClip.h"

#include <glm/gtc/packing.hpp>

#include <stdexcept>

namespace core {

void FlowClip::addFrame(const std::vector<glm::vec3>& velocity, const std::vector<float>& pressure)
{
    const std::size_t n = static_cast<std::size_t>(dims.x) * dims.y * dims.z;
    if (velocity.size() != n || pressure.size() != n)
        throw std::runtime_error("FlowClip: frame does not match the clip's grid");
    std::vector<std::uint64_t> frame(n);
    for (std::size_t c = 0; c < n; ++c)
        frame[c] = glm::packHalf4x16(glm::vec4(velocity[c], pressure[c]));
    frames.push_back(std::move(frame));
}

void FlowClip::decodeFrame(int index, float freestream, std::vector<glm::vec4>& out) const
{
    const std::vector<std::uint64_t>& frame = frames.at(static_cast<std::size_t>(index));
    const float s = freestreamSpeed > 0.0f ? freestream / freestreamSpeed : 1.0f;
    const glm::vec4 scale(s, s, s, s * s);
    out.resize(frame.size());
    for (std::size_t c = 0; c < frame.size(); ++c)
        out[c] = glm::unpackHalf4x16(frame[c]) * scale;
}

std::size_t FlowClip::bytes() const
{
    std::size_t total = 0;
    for (const auto& f : frames)
        total += f.size() * sizeof(std::uint64_t);
    return total;
}

} // namespace core
