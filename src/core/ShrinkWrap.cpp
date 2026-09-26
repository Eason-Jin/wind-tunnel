#include "core/ShrinkWrap.h"

#include "core/MarchingCubesTables.h"

#include <algorithm>
#include <utility>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace core {

namespace {

struct Grid {
    glm::ivec3 dims{0};
    glm::vec3 origin{0.0f}; // centre of voxel (0,0,0)
    float h = 1.0f;
    std::size_t size() const { return static_cast<std::size_t>(dims.x) * dims.y * dims.z; }
    std::size_t index(int i, int j, int k) const
    {
        return static_cast<std::size_t>(i) + static_cast<std::size_t>(dims.x) * (static_cast<std::size_t>(j) + static_cast<std::size_t>(dims.y) * k);
    }
};

// Mark every voxel a triangle passes through (dense barycentric sampling at
// half-voxel spacing, so thin panels leave no pinholes).
void rasterise(const SurfaceMesh& mesh, const Grid& g, std::vector<std::uint8_t>& occ)
{
    const float step = 0.5f * g.h;
    for (std::size_t t = 0; t + 2 < mesh.positions.size(); t += 3) {
        const glm::vec3 a = mesh.positions[t], b = mesh.positions[t + 1], c = mesh.positions[t + 2];
        const float lab = glm::length(b - a), lac = glm::length(c - a), lbc = glm::length(c - b);
        const int n = std::max(1, static_cast<int>(std::ceil(std::max({lab, lac, lbc}) / step)));
        for (int i = 0; i <= n; ++i)
            for (int j = 0; j <= n - i; ++j) {
                const float u = static_cast<float>(i) / n, v = static_cast<float>(j) / n;
                const glm::vec3 p = a + u * (b - a) + v * (c - a);
                const glm::ivec3 q = glm::ivec3(glm::round((p - g.origin) / g.h));
                if (glm::all(glm::greaterThanEqual(q, glm::ivec3(0))) && glm::all(glm::lessThan(q, g.dims)))
                    occ[g.index(q.x, q.y, q.z)] = 1;
            }
    }
}

// Separable max filter (6 passes of radius r along x/y/z): dilation with a cube.
void dilate(const Grid& g, std::vector<std::uint8_t>& v, int r)
{
    std::vector<std::uint8_t> tmp(v.size());
    const glm::ivec3 d = g.dims;
    for (int axis = 0; axis < 3; ++axis) {
        for (int k = 0; k < d.z; ++k)
            for (int j = 0; j < d.y; ++j)
                for (int i = 0; i < d.x; ++i) {
                    std::uint8_t m = 0;
                    for (int o = -r; o <= r && !m; ++o) {
                        glm::ivec3 q(i, j, k);
                        q[axis] += o;
                        if (q[axis] >= 0 && q[axis] < d[axis])
                            m = v[g.index(q.x, q.y, q.z)];
                    }
                    tmp[g.index(i, j, k)] = m;
                }
        v.swap(tmp);
    }
}

// Flood-fill the voxels reachable from the grid boundary without crossing
// `wall`; returns 1 for reachable (outside) voxels.
std::vector<std::uint8_t> outsideOf(const Grid& g, const std::vector<std::uint8_t>& wall)
{
    std::vector<std::uint8_t> out(wall.size(), 0);
    std::vector<std::size_t> stack;
    const glm::ivec3 d = g.dims;
    auto seed = [&](int i, int j, int k) {
        const std::size_t n = g.index(i, j, k);
        if (!wall[n] && !out[n]) {
            out[n] = 1;
            stack.push_back(n);
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
    const std::size_t sx = 1, sy = static_cast<std::size_t>(d.x), sz = sy * static_cast<std::size_t>(d.y);
    while (!stack.empty()) {
        const std::size_t n = stack.back();
        stack.pop_back();
        const int i = static_cast<int>(n % sy), j = static_cast<int>((n / sy) % static_cast<std::size_t>(d.y)),
                  k = static_cast<int>(n / sz);
        auto visit = [&](bool ok, std::size_t m) {
            if (ok && !wall[m] && !out[m]) {
                out[m] = 1;
                stack.push_back(m);
            }
        };
        visit(i > 0, n - sx);
        visit(i < d.x - 1, n + sx);
        visit(j > 0, n - sy);
        visit(j < d.y - 1, n + sy);
        visit(k > 0, n - sz);
        visit(k < d.z - 1, n + sz);
    }
    return out;
}

// Taubin lambda/mu smoothing on the welded mesh: alternating shrink and
// inflate Laplacian steps remove high-frequency terracing without the volume
// loss of plain Laplacian smoothing.
std::size_t taubinSmooth(SurfaceMesh& mesh, int iterations)
{
    // Weld: marching-cubes vertices on a shared cube edge are bit-identical.
    std::unordered_map<std::uint64_t, std::uint32_t> ids;
    std::vector<glm::vec3> verts;
    std::vector<std::uint32_t> tri(mesh.positions.size());
    auto key = [](const glm::vec3& p) {
        std::uint64_t h = 1469598103934665603ull;
        for (int i = 0; i < 3; ++i) {
            std::uint32_t bits;
            std::memcpy(&bits, &p[i], sizeof bits);
            h = (h ^ bits) * 1099511628211ull;
        }
        return h;
    };
    for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
        const auto [it, inserted] = ids.try_emplace(key(mesh.positions[i]), static_cast<std::uint32_t>(verts.size()));
        if (inserted)
            verts.push_back(mesh.positions[i]);
        tri[i] = it->second;
    }
    // Vertex adjacency (unique neighbours).
    std::vector<std::vector<std::uint32_t>> nbrs(verts.size());
    for (std::size_t t = 0; t + 2 < tri.size(); t += 3)
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                if (a != b)
                    nbrs[tri[t + a]].push_back(tri[t + b]);
    for (auto& n : nbrs) {
        std::sort(n.begin(), n.end());
        n.erase(std::unique(n.begin(), n.end()), n.end());
    }
    std::vector<glm::vec3> next(verts.size());
    auto step = [&](float factor) {
        for (std::size_t v = 0; v < verts.size(); ++v) {
            if (nbrs[v].empty()) {
                next[v] = verts[v];
                continue;
            }
            glm::vec3 avg(0.0f);
            for (std::uint32_t n : nbrs[v])
                avg += verts[n];
            avg /= static_cast<float>(nbrs[v].size());
            next[v] = verts[v] + factor * (avg - verts[v]);
        }
        verts.swap(next);
    };
    for (int i = 0; i < iterations; ++i) {
        step(0.5f);
        step(-0.53f);
    }
    for (std::size_t i = 0; i < tri.size(); ++i)
        mesh.positions[i] = verts[tri[i]];
    return verts.size();
}

} // namespace

SurfaceMesh shrinkWrap(const SurfaceMesh& in, const ShrinkWrapOptions& options, const std::function<void(const std::string&)>& log)
{
    auto say = [&](const std::string& s) {
        if (log)
            log(s);
    };
    SurfaceMesh out;
    if (in.empty())
        return out;

    const float h = std::max(options.voxelSize, 1e-6f);
    const int pad = options.closeRadius + options.smoothPasses + 2;
    const Bounds b = in.bounds();
    Grid g;
    g.h = h;
    g.origin = b.min - glm::vec3(static_cast<float>(pad) * h);
    g.dims = glm::ivec3(glm::ceil(b.size() / h)) + glm::ivec3(2 * pad + 1);
    say("grid " + std::to_string(g.dims.x) + "x" + std::to_string(g.dims.y) + "x" + std::to_string(g.dims.z));

    std::vector<std::uint8_t> wall(g.size(), 0);
    rasterise(in, g, wall);
    say("rasterised surface");

    // Closing = dilate the walls, fill, then grow the outside back by the same
    // radius so the hull returns to the original size but gaps stay sealed.
    // The original panel voxels always stay solid: growing the outside back
    // must trim only the sealing material, never eat through thin panels.
    const std::vector<std::uint8_t> panels = wall;
    dilate(g, wall, options.closeRadius);
    std::vector<std::uint8_t> outside = outsideOf(g, wall);
    dilate(g, outside, options.closeRadius);
    say("sealed and filled");

    std::vector<float> field(g.size());
    for (std::size_t n = 0; n < field.size(); ++n)
        field[n] = (outside[n] && !panels[n]) ? 0.0f : 1.0f;
    // Light box blur so marching cubes produces smooth, sloped faces instead
    // of voxel stairs.
    const glm::ivec3 d = g.dims;
    std::vector<float> tmp(field.size());
    for (int pass = 0; pass < options.smoothPasses; ++pass)
        for (int axis = 0; axis < 3; ++axis) {
            for (int k = 0; k < d.z; ++k)
                for (int j = 0; j < d.y; ++j)
                    for (int i = 0; i < d.x; ++i) {
                        float sum = 0.0f;
                        int cnt = 0;
                        for (int o = -1; o <= 1; ++o) {
                            glm::ivec3 q(i, j, k);
                            q[axis] += o;
                            if (q[axis] >= 0 && q[axis] < d[axis]) {
                                sum += field[g.index(q.x, q.y, q.z)];
                                ++cnt;
                            }
                        }
                        tmp[g.index(i, j, k)] = sum / static_cast<float>(cnt);
                    }
            field.swap(tmp);
        }

    // Marching cubes at 0.5 over voxel centres.
    const float iso = 0.5f;
    for (int k = 0; k + 1 < d.z; ++k)
        for (int j = 0; j + 1 < d.y; ++j)
            for (int i = 0; i + 1 < d.x; ++i) {
                float val[8];
                int cube = 0;
                for (int c = 0; c < 8; ++c) {
                    const glm::ivec3 q = glm::ivec3(i, j, k) + kMcCornerOffset[c];
                    val[c] = field[g.index(q.x, q.y, q.z)];
                    if (val[c] > iso)
                        cube |= 1 << c;
                }
                if (kMcEdgeTable[cube] == 0)
                    continue;
                glm::vec3 edgePos[12];
                for (int e = 0; e < 12; ++e) {
                    if (!(kMcEdgeTable[cube] & (1 << e)))
                        continue;
                    int a = kMcEdgeCorner[e][0], c2 = kMcEdgeCorner[e][1];
                    // Interpolate each edge in a fixed direction (lower grid
                    // corner first) so neighbouring cubes produce bit-identical
                    // vertices, which lets the smoothing pass weld them.
                    const glm::ivec3 oa = kMcCornerOffset[a], ob = kMcCornerOffset[c2];
                    if (oa.x + oa.y + oa.z > ob.x + ob.y + ob.z)
                        std::swap(a, c2);
                    const float t = (iso - val[a]) / (val[c2] - val[a]);
                    const glm::vec3 pa = g.origin + h * glm::vec3(glm::ivec3(i, j, k) + kMcCornerOffset[a]);
                    const glm::vec3 pb = g.origin + h * glm::vec3(glm::ivec3(i, j, k) + kMcCornerOffset[c2]);
                    edgePos[e] = pa + t * (pb - pa);
                }
                for (int t = 0; kMcTriTable[cube][t] != -1; t += 3) {
                    // Table winding faces the low (outside) side; emit so the
                    // normal points out of the solid.
                    out.positions.push_back(edgePos[kMcTriTable[cube][t]]);
                    out.positions.push_back(edgePos[kMcTriTable[cube][t + 2]]);
                    out.positions.push_back(edgePos[kMcTriTable[cube][t + 1]]);
                }
            }
    if (options.taubinIterations > 0) {
        const std::size_t welded = taubinSmooth(out, options.taubinIterations);
        say("smoothed surface (" + std::to_string(welded) + " welded vertices from " + std::to_string(out.positions.size()) + ")");
    }
    out.computeFaceNormals();
    out.name = in.name.empty() ? "hull" : in.name + "_hull";
    say("hull has " + std::to_string(out.triangleCount()) + " triangles");
    return out;
}

} // namespace core
