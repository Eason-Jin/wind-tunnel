#include "core/Voxelize.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace core {

std::vector<std::uint8_t> voxelizeSolidMask(const SurfaceMesh& body, const glm::vec3& gridOrigin, const glm::vec3& spacing,
                                            const glm::ivec3& dims)
{
    const std::size_t cells = static_cast<std::size_t>(std::max(dims.x, 0)) * std::max(dims.y, 0) * std::max(dims.z, 0);
    std::vector<std::uint8_t> solid(cells, 0);
    if (cells == 0 || body.empty())
        return solid;
    // Work on a grid twice as fine as the target, then downsample: a cell is
    // solid when most of its 8 sub-cells are. Marking whole surface cells as
    // solid would otherwise fatten the body by up to one cell.
    constexpr int R = 2;
    const glm::ivec3 d = dims * R;
    const glm::vec3 sp = spacing / static_cast<float>(R);
    const glm::vec3 origin = gridOrigin - 0.5f * spacing + 0.5f * sp; // centre of sub-cell 0
    const std::size_t sy = static_cast<std::size_t>(d.x), sz = sy * static_cast<std::size_t>(d.y);
    const std::size_t n = sz * static_cast<std::size_t>(d.z);
    auto index = [&](int i, int j, int k) { return static_cast<std::size_t>(i) + sy * static_cast<std::size_t>(j) + sz * static_cast<std::size_t>(k); };

    // 1. Mark every sub-cell a triangle passes through (dense barycentric
    //    sampling at a third of a sub-cell, so none is missed).
    std::vector<std::uint8_t> wall(n, 0);
    const float step = std::min({sp.x, sp.y, sp.z}) / 3.0f;
    for (std::size_t t = 0; t + 2 < body.positions.size(); t += 3) {
        const glm::vec3 a = body.positions[t], b = body.positions[t + 1], c = body.positions[t + 2];
        const float longest = std::max({glm::length(b - a), glm::length(c - a), glm::length(c - b)});
        const int m = std::max(1, static_cast<int>(std::ceil(longest / step)));
        for (int i = 0; i <= m; ++i)
            for (int j = 0; j <= m - i; ++j) {
                const glm::vec3 p = a + (static_cast<float>(i) / m) * (b - a) + (static_cast<float>(j) / m) * (c - a);
                const glm::ivec3 q = glm::ivec3(glm::round((p - origin) / sp));
                if (glm::all(glm::greaterThanEqual(q, glm::ivec3(0))) && glm::all(glm::lessThan(q, d)))
                    wall[index(q.x, q.y, q.z)] = 1;
            }
    }

    // 2. Flood-fill the air from the grid boundary without crossing surface
    //    sub-cells. Gaps between panels narrower than a sub-cell cannot leak,
    //    so this also works for open, multi-part "game" meshes.
    std::vector<std::uint8_t> air(n, 0);
    std::vector<std::size_t> stack;
    auto seed = [&](int i, int j, int k) {
        const std::size_t c = index(i, j, k);
        if (!wall[c] && !air[c]) {
            air[c] = 1;
            stack.push_back(c);
        }
    };
    for (int k = 0; k < d.z; ++k)
        for (int j = 0; j < d.y; ++j) {
            seed(0, j, k);
            seed(d.x - 1, j, k);
        }
    for (int k = 0; k < d.z; ++k)
        for (int i = 0; i < d.x; ++i) {
            seed(i, 0, k);
            seed(i, d.y - 1, k);
        }
    for (int j = 0; j < d.y; ++j)
        for (int i = 0; i < d.x; ++i) {
            seed(i, j, 0);
            seed(i, j, d.z - 1);
        }
    while (!stack.empty()) {
        const std::size_t c = stack.back();
        stack.pop_back();
        const int i = static_cast<int>(c % sy), j = static_cast<int>((c / sy) % static_cast<std::size_t>(d.y)),
                  k = static_cast<int>(c / sz);
        auto visit = [&](bool ok, std::size_t nb) {
            if (ok && !wall[nb] && !air[nb]) {
                air[nb] = 1;
                stack.push_back(nb);
            }
        };
        visit(i > 0, c - 1);
        visit(i < d.x - 1, c + 1);
        visit(j > 0, c - sy);
        visit(j < d.y - 1, c + sy);
        visit(k > 0, c - sz);
        visit(k < d.z - 1, c + sz);
    }

    // 3. Downsample: a cell is solid when most of its sub-cells are
    //    unreachable by air (surface or enclosed volume).
    const std::size_t ny = static_cast<std::size_t>(dims.y), nx = static_cast<std::size_t>(dims.x);
    for (int k = 0; k < dims.z; ++k)
        for (int j = 0; j < dims.y; ++j)
            for (int i = 0; i < dims.x; ++i) {
                int solidSubs = 0;
                for (int dk = 0; dk < R; ++dk)
                    for (int dj = 0; dj < R; ++dj)
                        for (int di = 0; di < R; ++di)
                            solidSubs += !air[index(i * R + di, j * R + dj, k * R + dk)];
                if (2 * solidSubs > R * R * R) // strictly more than half
                    solid[static_cast<std::size_t>(i) + nx * (static_cast<std::size_t>(j) + ny * static_cast<std::size_t>(k))] = 1;
            }
    return solid;
}

void voxelizeSolid(const SurfaceMesh& body, FlowField& field)
{
    if (field.empty() || body.empty())
        return;
    field.solid = voxelizeSolidMask(body, field.origin, field.spacing, field.dims);
    for (std::size_t c = 0; c < field.solid.size(); ++c)
        if (field.solid[c])
            field.velocity[c] = glm::vec3(0.0f);
}

} // namespace core
