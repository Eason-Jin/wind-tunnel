#include "core/SurfaceMesh.h"

#include <glm/gtc/constants.hpp>

#include <limits>

namespace core {

Bounds SurfaceMesh::bounds() const
{
    if (positions.empty())
        return {};
    Bounds b{glm::vec3(std::numeric_limits<float>::max()), glm::vec3(std::numeric_limits<float>::lowest())};
    for (const auto& p : positions) {
        b.min = glm::min(b.min, p);
        b.max = glm::max(b.max, p);
    }
    return b;
}

void SurfaceMesh::computeFaceNormals()
{
    normals.resize(positions.size());
    for (std::size_t i = 0; i + 2 < positions.size(); i += 3) {
        const glm::vec3 n = glm::cross(positions[i + 1] - positions[i], positions[i + 2] - positions[i]);
        const float len = glm::length(n);
        const glm::vec3 unit = len > 0.0f ? n / len : glm::vec3(0.0f);
        normals[i] = normals[i + 1] = normals[i + 2] = unit;
    }
}

void SurfaceMesh::transform(float scale, const glm::vec3& translate)
{
    for (auto& p : positions)
        p = p * scale + translate;
}

SurfaceMesh makeSphereMesh(const glm::vec3& centre, float radius, int slices, int stacks)
{
    SurfaceMesh mesh;
    mesh.name = "sphere";
    auto point = [&](int i, int j) {
        const float theta = glm::pi<float>() * static_cast<float>(j) / static_cast<float>(stacks); // 0..pi from +z
        const float phi = glm::two_pi<float>() * static_cast<float>(i) / static_cast<float>(slices);
        return centre + radius * glm::vec3(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
    };
    for (int j = 0; j < stacks; ++j) {
        for (int i = 0; i < slices; ++i) {
            const glm::vec3 a = point(i, j), b = point(i + 1, j), c = point(i + 1, j + 1), d = point(i, j + 1);
            if (j != 0) { // skip degenerate triangle at the north pole
                mesh.positions.insert(mesh.positions.end(), {a, d, b});
            }
            if (j != stacks - 1) { // skip degenerate triangle at the south pole
                mesh.positions.insert(mesh.positions.end(), {b, d, c});
            }
        }
    }
    mesh.computeFaceNormals();
    return mesh;
}

} // namespace core
