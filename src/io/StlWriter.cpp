#include "io/StlLoader.h"

#include <fstream>
#include <stdexcept>

namespace io {

void writeStl(const core::SurfaceMesh& mesh, const std::filesystem::path& path, const std::string& solidName)
{
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("Cannot write STL file: " + path.string());
    out.precision(9);
    out << "solid " << solidName << '\n';
    for (std::size_t t = 0; t < mesh.triangleCount(); ++t) {
        const glm::vec3& a = mesh.positions[3 * t];
        const glm::vec3& b = mesh.positions[3 * t + 1];
        const glm::vec3& c = mesh.positions[3 * t + 2];
        glm::vec3 n = glm::cross(b - a, c - a);
        const float len = glm::length(n);
        n = len > 0.0f ? n / len : glm::vec3(0.0f);
        out << "  facet normal " << n.x << ' ' << n.y << ' ' << n.z << "\n    outer loop\n";
        for (const glm::vec3* v : {&a, &b, &c})
            out << "      vertex " << v->x << ' ' << v->y << ' ' << v->z << '\n';
        out << "    endloop\n  endfacet\n";
    }
    out << "endsolid " << solidName << '\n';
    if (!out)
        throw std::runtime_error("Failed while writing STL file: " + path.string());
}

} // namespace io
