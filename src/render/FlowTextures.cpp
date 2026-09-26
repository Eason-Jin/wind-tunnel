#include "render/FlowTextures.h"

#include <algorithm>
#include <vector>

namespace render {

void FlowTextures::upload(const core::FlowField& field)
{
    clear();
    if (field.empty())
        return;
    dims_ = field.dims;
    const core::Bounds b = field.bounds();
    gridMin_ = b.min;
    gridSize_ = b.size();
    maxSpeed_ = std::max(field.maxSpeed(), 1e-6f);
    field.pressureRange(pMin_, pMax_);
    freestream_ = field.freestreamSpeed;

    std::vector<glm::vec4> texels(field.cellCount());
    for (std::size_t n = 0; n < texels.size(); ++n)
        texels[n] = glm::vec4(field.velocity[n], field.pressure.empty() ? 0.0f : field.pressure[n]);

    flow_ = gl::createTexture(GL_TEXTURE_3D);
    glTextureStorage3D(flow_.id(), 1, GL_RGBA32F, dims_.x, dims_.y, dims_.z);
    glTextureSubImage3D(flow_.id(), 0, 0, 0, 0, dims_.x, dims_.y, dims_.z, GL_RGBA, GL_FLOAT, texels.data());
    glTextureParameteri(flow_.id(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTextureParameteri(flow_.id(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    for (GLenum wrap : {GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_TEXTURE_WRAP_R})
        glTextureParameteri(flow_.id(), wrap, GL_CLAMP_TO_EDGE);

    std::vector<std::uint8_t> solid = field.solid;
    solid.resize(field.cellCount(), 0);
    for (auto& s : solid)
        s = s ? 255 : 0;
    solid_ = gl::createTexture(GL_TEXTURE_3D);
    glTextureStorage3D(solid_.id(), 1, GL_R8, dims_.x, dims_.y, dims_.z);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTextureSubImage3D(solid_.id(), 0, 0, 0, 0, dims_.x, dims_.y, dims_.z, GL_RED, GL_UNSIGNED_BYTE, solid.data());
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    // Linear filtering: shaders threshold at 0.5, which gives a smooth body
    // outline instead of a voxel staircase.
    glTextureParameteri(solid_.id(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTextureParameteri(solid_.id(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    for (GLenum wrap : {GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_TEXTURE_WRAP_R})
        glTextureParameteri(solid_.id(), wrap, GL_CLAMP_TO_EDGE);
}

void FlowTextures::clear()
{
    flow_.reset();
    solid_.reset();
    dims_ = glm::ivec3(0);
}

void FlowTextures::bind() const
{
    glBindTextureUnit(kFlowUnit, flow_.id());
    glBindTextureUnit(kSolidUnit, solid_.id());
}

void FlowTextures::setUniforms(gl::Shader& shader) const
{
    shader.set("uFlow", static_cast<int>(kFlowUnit));
    shader.set("uSolid", static_cast<int>(kSolidUnit));
    shader.set("uGridMin", gridMin_);
    shader.set("uGridSize", gridSize_);
    shader.set("uGridDims", dims_);
    shader.set("uMaxSpeed", maxSpeed_);
    shader.set("uPressureMin", pMin_);
    shader.set("uPressureMax", pMax_);
    shader.set("uFreestream", freestream_);
}

} // namespace render
