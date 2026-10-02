#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace core {

// Snapshots of an unsteady flow on the same grid as its FlowField, for
// playing the shedding eddies back as an animation (the FlowField itself
// holds the time average). Each cell is four half floats packed into 64 bits
// (velocity x, y, z in m/s and kinematic pressure), so a frame uploads
// straight into a 3D texture and a Fine-quality clip fits in a few hundred MB.
struct FlowClip {
    glm::ivec3 dims{0};
    float freestreamSpeed = 1.0f; // speed the frames were recorded at
    float frameSeconds = 0.0f;    // simulated time between frames
    std::vector<std::vector<std::uint64_t>> frames;

    int frameCount() const { return static_cast<int>(frames.size()); }

    // Pack one snapshot (cell-for-cell with `dims`) and append it.
    void addFrame(const std::vector<glm::vec3>& velocity, const std::vector<float>& pressure);

    // Frame `index` as (velocity, pressure) per cell, for a free stream of
    // `freestream` m/s: velocities scale linearly and pressures with its
    // square, as for the time-averaged field.
    void decodeFrame(int index, float freestream, std::vector<glm::vec4>& out) const;

    std::size_t bytes() const;
};

} // namespace core
