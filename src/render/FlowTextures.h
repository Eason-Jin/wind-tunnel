#pragma once

#include "core/FlowField.h"
#include "render/gl/GlObjects.h"
#include "render/gl/Shader.h"

namespace render {

// GPU copy of a FlowField, shared by all passes:
//   flow  : 3D RGBA32F, rgb = velocity (m/s), a = kinematic pressure; linear filtering
//   solid : 3D R8, 1 inside the body; nearest filtering
// Texture coordinates for a world position p: (p - boundsMin) / boundsSize
// (see setUniforms / FlowField::toTexCoord).
class FlowTextures {
public:
    // Texture units reserved for the flow data. Passes should use units >= 4
    // for their own textures (e.g. the colormap).
    static constexpr GLuint kFlowUnit = 0;
    static constexpr GLuint kSolidUnit = 1;

    void upload(const core::FlowField& field);
    void clear();
    bool valid() const { return static_cast<bool>(flow_); }

    GLuint flowTexture() const { return flow_.id(); }
    GLuint solidTexture() const { return solid_.id(); }

    // Bind to kFlowUnit / kSolidUnit.
    void bind() const;

    // Sets on `shader`: sampler3D uFlow (unit 0), sampler3D uSolid (unit 1),
    // vec3 uGridMin, vec3 uGridSize (world bounds of the grid), ivec3 uGridDims,
    // float uMaxSpeed, float uPressureMin, float uPressureMax, float uFreestream.
    void setUniforms(gl::Shader& shader) const;

    glm::vec3 gridMin() const { return gridMin_; }
    glm::vec3 gridSize() const { return gridSize_; }
    glm::ivec3 dims() const { return dims_; }
    float maxSpeed() const { return maxSpeed_; }
    float pressureMin() const { return pMin_; }
    float pressureMax() const { return pMax_; }
    float freestream() const { return freestream_; }

private:
    gl::Texture flow_;
    gl::Texture solid_;
    glm::vec3 gridMin_{0.0f}, gridSize_{1.0f};
    glm::ivec3 dims_{0};
    float maxSpeed_ = 1.0f, pMin_ = 0.0f, pMax_ = 1.0f, freestream_ = 1.0f;
};

} // namespace render
