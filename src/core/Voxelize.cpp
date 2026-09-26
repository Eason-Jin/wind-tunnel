#include "core/Voxelize.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace core {

void voxelizeSolid(const SurfaceMesh& body, FlowField& field)
{
    if (field.empty() || body.empty())
        return;
    const glm::ivec3 d = field.dims;
    field.solid.assign(field.cellCount(), 0);

    // Bin triangles by the (j,k) columns their yz bounding box overlaps.
    std::vector<std::vector<int>> bins(static_cast<std::size_t>(d.y) * d.z);
    auto toCol = [&](float v, int axis) { return (v - field.origin[axis]) / field.spacing[axis]; };
    const int triCount = static_cast<int>(body.triangleCount());
    for (int t = 0; t < triCount; ++t) {
        const glm::vec3& a = body.positions[3 * t];
        const glm::vec3& b = body.positions[3 * t + 1];
        const glm::vec3& c = body.positions[3 * t + 2];
        const int j0 = std::max(0, static_cast<int>(std::ceil(toCol(std::min({a.y, b.y, c.y}), 1))));
        const int j1 = std::min(d.y - 1, static_cast<int>(std::floor(toCol(std::max({a.y, b.y, c.y}), 1))));
        const int k0 = std::max(0, static_cast<int>(std::ceil(toCol(std::min({a.z, b.z, c.z}), 2))));
        const int k1 = std::min(d.z - 1, static_cast<int>(std::floor(toCol(std::max({a.z, b.z, c.z}), 2))));
        for (int k = k0; k <= k1; ++k)
            for (int j = j0; j <= j1; ++j)
                bins[static_cast<std::size_t>(k) * d.y + j].push_back(t);
    }

    std::vector<float> hits;
    for (int k = 0; k < d.z; ++k) {
        for (int j = 0; j < d.y; ++j) {
            const auto& bin = bins[static_cast<std::size_t>(k) * d.y + j];
            if (bin.empty())
                continue;
            // Nudge the ray off exact grid values to avoid hitting shared edges twice.
            const float y = field.origin.y + field.spacing.y * j + 1.37e-5f * field.spacing.y;
            const float z = field.origin.z + field.spacing.z * k + 2.91e-5f * field.spacing.z;
            hits.clear();
            for (int t : bin) {
                const glm::vec3& a = body.positions[3 * t];
                const glm::vec3& b = body.positions[3 * t + 1];
                const glm::vec3& c = body.positions[3 * t + 2];
                // Barycentric test in the yz plane.
                const float det = (b.y - a.y) * (c.z - a.z) - (c.y - a.y) * (b.z - a.z);
                if (std::abs(det) < 1e-20f)
                    continue;
                const float u = ((y - a.y) * (c.z - a.z) - (c.y - a.y) * (z - a.z)) / det;
                const float v = ((b.y - a.y) * (z - a.z) - (y - a.y) * (b.z - a.z)) / det;
                if (u < 0.0f || v < 0.0f || u + v > 1.0f)
                    continue;
                hits.push_back(a.x + u * (b.x - a.x) + v * (c.x - a.x));
            }
            if (hits.size() < 2 || hits.size() % 2 != 0)
                continue;
            std::sort(hits.begin(), hits.end());
            for (std::size_t h = 0; h + 1 < hits.size(); h += 2) {
                const int i0 = std::max(0, static_cast<int>(std::ceil(toCol(hits[h], 0))));
                const int i1 = std::min(d.x - 1, static_cast<int>(std::floor(toCol(hits[h + 1], 0))));
                for (int i = i0; i <= i1; ++i) {
                    const std::size_t n = field.index(i, j, k);
                    field.solid[n] = 1;
                    field.velocity[n] = glm::vec3(0.0f);
                }
            }
        }
    }
}

} // namespace core
