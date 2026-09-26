#include "core/FlowField.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace core {

Bounds FlowField::bounds() const
{
    const glm::vec3 half = 0.5f * spacing;
    return {origin - half, origin + spacing * glm::vec3(dims - 1) + half};
}

glm::vec3 FlowField::toTexCoord(const glm::vec3& p) const
{
    const Bounds b = bounds();
    return (p - b.min) / (b.max - b.min);
}

void FlowField::resize(const glm::ivec3& newDims)
{
    dims = newDims;
    const std::size_t n = cellCount();
    velocity.assign(n, glm::vec3(0.0f));
    pressure.assign(n, 0.0f);
    solid.assign(n, 0);
}

bool FlowField::contains(const glm::vec3& p) const
{
    return !empty() && bounds().contains(p);
}

namespace {

// Continuous cell coordinate, clamped so that i0 and i0+1 are both valid.
struct Lerp3 {
    int i0, j0, k0;
    glm::vec3 t;
};

Lerp3 lerpCoords(const FlowField& f, const glm::vec3& p)
{
    glm::vec3 c = (p - f.origin) / f.spacing;
    const glm::vec3 maxC = glm::vec3(f.dims - 1);
    c = glm::clamp(c, glm::vec3(0.0f), maxC);
    glm::ivec3 i0 = glm::ivec3(glm::floor(c));
    i0 = glm::clamp(i0, glm::ivec3(0), glm::max(f.dims - 2, glm::ivec3(0)));
    return {i0.x, i0.y, i0.z, c - glm::vec3(i0)};
}

template <typename T>
T trilinear(const FlowField& f, const std::vector<T>& data, const glm::vec3& p)
{
    const Lerp3 l = lerpCoords(f, p);
    const int i1 = std::min(l.i0 + 1, f.dims.x - 1);
    const int j1 = std::min(l.j0 + 1, f.dims.y - 1);
    const int k1 = std::min(l.k0 + 1, f.dims.z - 1);
    auto at = [&](int i, int j, int k) { return data[f.index(i, j, k)]; };
    const T c00 = at(l.i0, l.j0, l.k0) * (1.0f - l.t.x) + at(i1, l.j0, l.k0) * l.t.x;
    const T c10 = at(l.i0, j1, l.k0) * (1.0f - l.t.x) + at(i1, j1, l.k0) * l.t.x;
    const T c01 = at(l.i0, l.j0, k1) * (1.0f - l.t.x) + at(i1, l.j0, k1) * l.t.x;
    const T c11 = at(l.i0, j1, k1) * (1.0f - l.t.x) + at(i1, j1, k1) * l.t.x;
    const T c0 = c00 * (1.0f - l.t.y) + c10 * l.t.y;
    const T c1 = c01 * (1.0f - l.t.y) + c11 * l.t.y;
    return c0 * (1.0f - l.t.z) + c1 * l.t.z;
}

} // namespace

glm::vec3 FlowField::sampleVelocity(const glm::vec3& p) const
{
    if (!contains(p))
        return glm::vec3(0.0f);
    return trilinear(*this, velocity, p);
}

float FlowField::samplePressure(const glm::vec3& p) const
{
    if (!contains(p))
        return 0.0f;
    return trilinear(*this, pressure, p);
}

bool FlowField::isSolid(const glm::vec3& p) const
{
    if (!contains(p) || solid.empty())
        return false;
    glm::ivec3 c = glm::ivec3(glm::round((p - origin) / spacing));
    c = glm::clamp(c, glm::ivec3(0), dims - 1);
    return solid[index(c.x, c.y, c.z)] != 0;
}

float FlowField::maxSpeed() const
{
    float m = 0.0f;
    for (const auto& v : velocity)
        m = std::max(m, glm::length(v));
    return m;
}

void FlowField::pressureRange(float& minOut, float& maxOut) const
{
    minOut = std::numeric_limits<float>::max();
    maxOut = std::numeric_limits<float>::lowest();
    for (std::size_t n = 0; n < pressure.size(); ++n) {
        if (!solid.empty() && solid[n])
            continue;
        minOut = std::min(minOut, pressure[n]);
        maxOut = std::max(maxOut, pressure[n]);
    }
    if (minOut > maxOut)
        minOut = maxOut = 0.0f;
}

} // namespace core
