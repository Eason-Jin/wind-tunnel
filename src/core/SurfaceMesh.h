#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace core {

struct Bounds {
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};

    glm::vec3 centre() const { return 0.5f * (min + max); }
    glm::vec3 size() const { return max - min; }
    float radius() const { return 0.5f * glm::length(size()); }
    bool contains(const glm::vec3& p) const
    {
        return glm::all(glm::greaterThanEqual(p, min)) && glm::all(glm::lessThanEqual(p, max));
    }
};

// Triangle soup: every 3 consecutive positions form one triangle. Normals are
// per vertex (flat meshes repeat the face normal three times).
struct SurfaceMesh {
    std::string name;
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    // Optional per-vertex scalar (e.g. surface pressure). Empty when unused.
    std::vector<float> scalar;

    std::size_t triangleCount() const { return positions.size() / 3; }
    bool empty() const { return positions.empty(); }

    Bounds bounds() const;

    // Replace normals with flat face normals computed from the winding.
    // Degenerate triangles get a zero normal.
    void computeFaceNormals();

    // Uniformly scale and translate all positions (normals are unaffected).
    void transform(float scale, const glm::vec3& translate);
};

// UV sphere, flat shaded. Used for testing without an STL file.
SurfaceMesh makeSphereMesh(const glm::vec3& centre, float radius, int slices = 48, int stacks = 24);

} // namespace core
