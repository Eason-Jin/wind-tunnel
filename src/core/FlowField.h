#pragma once

#include "core/SurfaceMesh.h"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace core {

// Flow solution resampled onto a uniform, axis-aligned grid. This is the only
// flow data the renderer sees, so any solver (OpenFOAM, a future CUDA solver,
// the analytic test field) must produce one.
//
// Layout: values live at cell centres; cell (i,j,k) has centre
// origin + spacing * (i,j,k). Flat index = i + dims.x * (j + dims.y * k),
// i.e. x varies fastest (matches OpenFOAM blockMesh cell order and GL 3D
// texture layout).
//
// World frame: flow travels along +x, +z is up.
struct FlowField {
    glm::ivec3 dims{0};
    glm::vec3 origin{0.0f};  // centre of cell (0,0,0)
    glm::vec3 spacing{1.0f}; // cell size
    float freestreamSpeed = 1.0f;

    std::vector<glm::vec3> velocity; // m/s
    std::vector<float> pressure;     // kinematic pressure p/rho (m^2/s^2), gauge
    std::vector<std::uint8_t> solid; // 1 = inside the body

    bool empty() const { return velocity.empty(); }
    std::size_t cellCount() const { return static_cast<std::size_t>(dims.x) * dims.y * dims.z; }
    std::size_t index(int i, int j, int k) const
    {
        return static_cast<std::size_t>(i) + static_cast<std::size_t>(dims.x) * (static_cast<std::size_t>(j) + static_cast<std::size_t>(dims.y) * k);
    }
    glm::vec3 cellCentre(int i, int j, int k) const { return origin + spacing * glm::vec3(i, j, k); }

    // Outer faces of the grid (cell centres +/- half a cell).
    Bounds bounds() const;

    // Map a world position to normalised [0,1]^3 texture coordinates, matching
    // GL_LINEAR sampling of a 3D texture built from this field.
    glm::vec3 toTexCoord(const glm::vec3& p) const;

    // Allocate storage for dims and zero it.
    void resize(const glm::ivec3& newDims);

    // Trilinear sampling at a world position. Positions outside the grid return
    // zero velocity / zero pressure. contains() tests the grid bounds.
    bool contains(const glm::vec3& p) const;
    glm::vec3 sampleVelocity(const glm::vec3& p) const;
    float samplePressure(const glm::vec3& p) const;
    bool isSolid(const glm::vec3& p) const; // nearest cell

    float maxSpeed() const;
    void pressureRange(float& minOut, float& maxOut) const;
};

} // namespace core
