#include "core/VortexCores.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>

namespace core {

std::vector<VortexCore> findVortexCores(const FlowField& field, const VortexCoreOptions& options)
{
    std::vector<VortexCore> cores;
    const glm::ivec3 d = field.dims;
    if (field.empty() || d.x < 3 || d.y < 3 || d.z < 3 || options.maxCores <= 0)
        return cores;
    const std::size_t n = field.cellCount();
    const float U = std::max(field.freestreamSpeed, 1e-4f);
    const float L = std::max(options.charLength, 1e-4f);
    const float qScale = (U / L) * (U / L);

    // Distance to the body in cells (capped), to drop the thin sheared layer
    // hugging the surface: it has high vorticity but is not a vortex.
    const auto gap = static_cast<std::uint8_t>(std::clamp(options.wallGap, 0, 250));
    std::vector<std::uint8_t> wallDist(n, 255);
    if (!field.solid.empty()) {
        std::queue<std::size_t> q;
        for (std::size_t c = 0; c < n; ++c)
            if (field.solid[c]) {
                wallDist[c] = 0;
                if (gap > 0)
                    q.push(c);
            }
        const std::size_t sy = static_cast<std::size_t>(d.x), sz = sy * static_cast<std::size_t>(d.y);
        while (!q.empty()) {
            const std::size_t c = q.front();
            q.pop();
            if (wallDist[c] >= gap)
                continue;
            const int i = static_cast<int>(c % sy), j = static_cast<int>((c / sy) % static_cast<std::size_t>(d.y)),
                      k = static_cast<int>(c / sz);
            const auto visit = [&](bool ok, std::size_t nb) {
                if (ok && wallDist[nb] > wallDist[c] + 1) {
                    wallDist[nb] = static_cast<std::uint8_t>(wallDist[c] + 1);
                    q.push(nb);
                }
            };
            visit(i > 0, c - 1);
            visit(i < d.x - 1, c + 1);
            visit(j > 0, c - sy);
            visit(j < d.y - 1, c + sy);
            visit(k > 0, c - sz);
            visit(k < d.z - 1, c + sz);
        }
    }

    // Q* and vorticity from central differences (interior cells only).
    std::vector<float> q(n, -1e9f);
    std::vector<glm::vec3> omega(n, glm::vec3(0.0f));
    const glm::vec3 h2 = 2.0f * field.spacing;
    for (int k = 1; k < d.z - 1; ++k)
        for (int j = 1; j < d.y - 1; ++j)
            for (int i = 1; i < d.x - 1; ++i) {
                const std::size_t c = field.index(i, j, k);
                if (wallDist[c] <= gap)
                    continue;
                const glm::vec3 dx = (field.velocity[field.index(i + 1, j, k)] - field.velocity[field.index(i - 1, j, k)]) / h2.x;
                const glm::vec3 dy = (field.velocity[field.index(i, j + 1, k)] - field.velocity[field.index(i, j - 1, k)]) / h2.y;
                const glm::vec3 dz = (field.velocity[field.index(i, j, k + 1)] - field.velocity[field.index(i, j, k - 1)]) / h2.z;
                const glm::mat3 J(dx, dy, dz); // J[col][row] = d(u_row)/d(x_col)
                float s2 = 0.0f, o2 = 0.0f;
                for (int a = 0; a < 3; ++a)
                    for (int b = 0; b < 3; ++b) {
                        const float s = 0.5f * (J[b][a] + J[a][b]);
                        const float o = 0.5f * (J[b][a] - J[a][b]);
                        s2 += s * s;
                        o2 += o * o;
                    }
                q[c] = 0.5f * (o2 - s2) / qScale;
                omega[c] = glm::vec3(dy.z - dz.y, dz.x - dx.z, dx.y - dy.x);
            }

    // Local maxima of Q* above the threshold are candidate core points.
    struct Candidate {
        float strength;
        std::size_t cell;
        glm::ivec3 ijk;
    };
    std::vector<Candidate> candidates;
    for (int k = 1; k < d.z - 1; ++k)
        for (int j = 1; j < d.y - 1; ++j)
            for (int i = 1; i < d.x - 1; ++i) {
                const std::size_t c = field.index(i, j, k);
                if (q[c] < options.minStrength || glm::length(omega[c]) < 1e-6f)
                    continue;
                bool peak = true;
                for (int dk = -1; dk <= 1 && peak; ++dk)
                    for (int dj = -1; dj <= 1 && peak; ++dj)
                        for (int di = -1; di <= 1 && peak; ++di)
                            if ((di || dj || dk) && q[field.index(i + di, j + dj, k + dk)] > q[c])
                                peak = false;
                if (peak)
                    candidates.push_back({q[c], c, {i, j, k}});
            }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.strength > b.strength; });

    // Strongest first, skipping points too close to one already chosen.
    const float minSep2 = options.minSeparation * options.minSeparation;
    for (const Candidate& cand : candidates) {
        const glm::vec3 p = field.cellCentre(cand.ijk.x, cand.ijk.y, cand.ijk.z);
        const bool tooClose = std::any_of(cores.begin(), cores.end(), [&](const VortexCore& other) {
            const glm::vec3 delta = other.position - p;
            return glm::dot(delta, delta) < minSep2;
        });
        if (tooClose)
            continue;
        cores.push_back({p, glm::normalize(omega[cand.cell]), cand.strength});
        if (static_cast<int>(cores.size()) >= options.maxCores)
            break;
    }
    return cores;
}

std::vector<glm::vec3> ringAround(const VortexCore& core, float radius, int count)
{
    std::vector<glm::vec3> points;
    if (count <= 0)
        return points;
    const glm::vec3 axis = glm::normalize(core.axis);
    // Any vector not parallel to the axis gives a perpendicular basis.
    const glm::vec3 helper = std::abs(axis.z) < 0.9f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 e1 = glm::normalize(glm::cross(axis, helper));
    const glm::vec3 e2 = glm::cross(axis, e1);
    points.reserve(static_cast<std::size_t>(count));
    for (int s = 0; s < count; ++s) {
        const float a = 6.2831853f * static_cast<float>(s) / static_cast<float>(count);
        points.push_back(core.position + radius * (std::cos(a) * e1 + std::sin(a) * e2));
    }
    return points;
}

} // namespace core
