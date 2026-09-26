#include "solvers/synthetic/SyntheticSolver.h"

#include "core/Voxelize.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace solvers {

core::FlowField makeSyntheticField(const core::SurfaceMesh& body, const core::SimulationParams& params)
{
    const core::Bounds b = body.bounds();
    const core::TunnelDomain domain = core::makeTunnelDomain(b, params);
    core::FlowField f = core::makeFieldForDomain(domain);
    f.freestreamSpeed = params.inletSpeed;

    const glm::vec3 c = b.centre();
    const glm::vec3 s = b.size();
    const float R = 0.5f * std::max({s.x, s.y, s.z, 1e-6f});
    const float U = params.inletSpeed;
    const float R3 = R * R * R;

    for (int k = 0; k < f.dims.z; ++k) {
        for (int j = 0; j < f.dims.y; ++j) {
            for (int i = 0; i < f.dims.x; ++i) {
                const std::size_t n = f.index(i, j, k);
                const glm::vec3 x = f.cellCentre(i, j, k) - c;
                const float r = glm::length(x);
                glm::vec3 u(0.0f);
                if (r > R) {
                    // u = grad(phi), phi = U x (1 + R^3 / (2 r^3))
                    const float r3 = r * r * r;
                    const float r5 = r3 * r * r;
                    u = U * (glm::vec3(1.0f + R3 / (2.0f * r3), 0.0f, 0.0f) - (1.5f * R3 / r5) * x.x * x);
                } else {
                    f.solid[n] = 1;
                }
                f.velocity[n] = u;
                f.pressure[n] = f.solid[n] ? 0.0f : 0.5f * (U * U - glm::dot(u, u));
            }
        }
    }
    core::voxelizeSolid(body, f);
    // Cells inside the analytic sphere stay solid as well (voxelizeSolid resets the mask).
    for (int k = 0; k < f.dims.z; ++k)
        for (int j = 0; j < f.dims.y; ++j)
            for (int i = 0; i < f.dims.x; ++i)
                if (glm::length(f.cellCentre(i, j, k) - c) <= R) {
                    const std::size_t n = f.index(i, j, k);
                    f.solid[n] = 1;
                    f.velocity[n] = glm::vec3(0.0f);
                    f.pressure[n] = 0.0f;
                }
    return f;
}

void SyntheticSolver::setup(const core::SurfaceMesh& body, const core::SimulationParams& params)
{
    if (body.empty())
        throw std::runtime_error("Synthetic solver: body mesh is empty");
    body_ = body;
    params_ = params;
}

void SyntheticSolver::run(const core::ProgressFn& progress, const std::atomic<bool>& cancel)
{
    if (progress)
        progress({"Solving", 0.0f, "Evaluating analytic potential flow"});
    if (cancel)
        return;
    field_ = makeSyntheticField(body_, params_);
    if (progress)
        progress({"Done", 1.0f, "Analytic field ready"});
}

} // namespace solvers
