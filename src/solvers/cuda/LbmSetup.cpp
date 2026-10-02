#include "solvers/cuda/LbmSetup.h"

#include "core/Voxelize.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace solvers::lbm {

LatticeUnits makeLatticeUnits(float dx, float inletSpeed, float kinematicViscosity, float uLattice)
{
    if (!(dx > 0.0f) || !(inletSpeed > 0.0f) || !(kinematicViscosity > 0.0f) || !(uLattice > 0.0f))
        throw std::runtime_error("LBM: lattice spacing, inlet speed and viscosity must be positive");
    LatticeUnits u;
    u.dx = dx;
    u.uLattice = uLattice;
    u.dt = dx * uLattice / inletSpeed;
    u.nuLattice = kinematicViscosity * u.dt / (dx * dx);
    u.tau0 = 3.0f * u.nuLattice + 0.5f;
    return u;
}

LatticeParams LbmPlan::latticeParams() const
{
    LatticeParams p;
    p.nx = lattice.x;
    p.ny = lattice.y;
    p.nz = lattice.z;
    p.uInlet = units.uLattice;
    p.tau0 = units.tau0;
    p.smagorinsky = kSmagorinsky;
    p.movingFloor = movingFloor;
    p.wallModel = true;
    return p;
}

std::size_t latticeDeviceBytes(std::size_t cells)
{
    return cells * (2 * kQ * sizeof(float) + sizeof(std::uint8_t) + 4 * sizeof(float));
}

LbmPlan makeLbmPlan(const core::Bounds& body, const core::SimulationParams& params)
{
    LbmPlan plan;
    plan.domain = core::makeTunnelDomain(body, params);
    plan.refine = std::clamp(params.lbmRefine, 1, 4);
    plan.lattice = plan.domain.cells * plan.refine;
    const float dx = plan.domain.cellSize / static_cast<float>(plan.refine);
    plan.latticeOrigin = plan.domain.box.min + glm::vec3(0.5f * dx);
    plan.units = makeLatticeUnits(dx, params.inletSpeed, params.kinematicViscosity);
    plan.movingFloor = params.groundPlane;

    // One flow-through is the time the free stream takes from inlet to outlet.
    const float flowThrough = static_cast<float>(plan.lattice.x) / plan.units.uLattice;
    plan.steps = std::max(100, static_cast<int>(std::ceil(std::max(params.lbmFlowThroughs, 0.1f) * flowThrough)));
    // The first 40% develops the flow (the start is impulsive); the rest is
    // averaged, which for the default run length is over a flow-through.
    plan.averageFrom = static_cast<int>(0.4f * static_cast<float>(plan.steps));

    // A clip of the end of the run for playback: the free stream moves about
    // 1.5 output cells between frames (smooth motion), with as many frames as
    // the memory budget allows (8 bytes per output cell per frame), all within
    // the developed part of the run.
    const std::size_t outputCells = static_cast<std::size_t>(plan.domain.cells.x) * plan.domain.cells.y * plan.domain.cells.z;
    plan.clipEvery = std::max(1, static_cast<int>(std::lround(1.5f * static_cast<float>(plan.refine) / plan.units.uLattice)));
    // Each snapshot averages the half frame spacing before it: the flow moves
    // under a cell in that time, so the eddies barely blur while faster
    // lattice noise averages out.
    plan.clipSmoothing = std::max(1, plan.clipEvery / 2);
    int frames = static_cast<int>(std::min<std::size_t>(kClipMaxFrames, kClipBudgetBytes / std::max<std::size_t>(8 * outputCells, 1)));
    frames = std::min(frames, (plan.steps - plan.averageFrom - plan.clipSmoothing) / plan.clipEvery + 1);
    plan.clipFrames = frames >= kClipMinFrames ? frames : 0;
    plan.clipFrom = plan.steps - (std::max(plan.clipFrames, 1) - 1) * plan.clipEvery;
    plan.deviceBytes = latticeDeviceBytes(plan.cells());
    return plan;
}

void closeFloorGaps(std::vector<std::uint8_t>& solid, const glm::ivec3& d, int cells)
{
    const std::size_t layer = static_cast<std::size_t>(d.x) * d.y;
    if (solid.size() != layer * d.z)
        throw std::runtime_error("LBM: solid mask does not match the lattice size");
    const int top = std::min(cells, d.z - 1);
    for (std::size_t c = 0; c < layer; ++c) {
        int lowest = -1;
        for (int z = 0; z <= top && lowest < 0; ++z)
            if (solid[c + layer * z])
                lowest = z;
        for (int z = 0; z < lowest; ++z)
            solid[c + layer * z] = 1;
    }
}

std::vector<std::uint8_t> buildFlags(const std::vector<std::uint8_t>& solid, const glm::ivec3& d)
{
    const std::size_t n = static_cast<std::size_t>(d.x) * d.y * d.z;
    if (solid.size() != n)
        throw std::runtime_error("LBM: solid mask does not match the lattice size");
    std::vector<std::uint8_t> flags(n, kFluid);
    auto at = [&](int x, int y, int z) {
        return static_cast<std::size_t>(x) + static_cast<std::size_t>(d.x) * (static_cast<std::size_t>(y) + static_cast<std::size_t>(d.y) * z);
    };
    for (int z = 0; z < d.z; ++z)
        for (int y = 0; y < d.y; ++y)
            for (int x = 0; x < d.x; ++x) {
                const std::size_t c = at(x, y, z);
                if (solid[c]) {
                    flags[c] = kSolid;
                    continue;
                }
                bool near = x == 0 || y == 0 || z == 0 || x == d.x - 1 || y == d.y - 1 || z == d.z - 1;
                for (int k = 1; k < kQ && !near; ++k)
                    near = solid[at(x - cx(k), y - cy(k), z - cz(k))] != 0;
                if (near)
                    flags[c] = kNearBoundary;
            }
    return flags;
}

std::vector<std::uint8_t> latticeFlags(const core::SurfaceMesh& body, const LbmPlan& plan)
{
    std::vector<std::uint8_t> solid = core::voxelizeSolidMask(body, plan.latticeOrigin, glm::vec3(plan.units.dx), plan.lattice);
    if (plan.movingFloor)
        closeFloorGaps(solid, plan.lattice, kFloorGapCells);
    return buildFlags(solid, plan.lattice);
}

void resampleMeans(const std::vector<float>& sums, int samples, const std::vector<std::uint8_t>& latticeFlags,
                   const glm::ivec3& lattice, int refine, const LatticeUnits& units, core::FlowField& field)
{
    const std::size_t n = static_cast<std::size_t>(lattice.x) * lattice.y * lattice.z;
    if (sums.size() != 4 * n || latticeFlags.size() != n)
        throw std::runtime_error("LBM: time-average buffers do not match the lattice size");
    if (field.dims * refine != lattice)
        throw std::runtime_error("LBM: output grid does not match the lattice");
    if (samples <= 0)
        throw std::runtime_error("LBM: the run finished before any time-average sample was taken");
    if (field.solid.size() != field.cellCount())
        field.solid.assign(field.cellCount(), 0);

    const float toVelocity = units.speedScale() / static_cast<float>(samples);
    for (int k = 0; k < field.dims.z; ++k)
        for (int j = 0; j < field.dims.y; ++j)
            for (int i = 0; i < field.dims.x; ++i) {
                const std::size_t o = field.index(i, j, k);
                double u[4] = {0.0, 0.0, 0.0, 0.0};
                int fluid = 0;
                for (int dk = 0; dk < refine; ++dk)
                    for (int dj = 0; dj < refine; ++dj)
                        for (int di = 0; di < refine; ++di) {
                            const std::size_t c = static_cast<std::size_t>(i * refine + di) +
                                                  static_cast<std::size_t>(lattice.x) *
                                                      (static_cast<std::size_t>(j * refine + dj) +
                                                       static_cast<std::size_t>(lattice.y) * static_cast<std::size_t>(k * refine + dk));
                            if (latticeFlags[c] & kSolid)
                                continue;
                            ++fluid;
                            for (int q = 0; q < 4; ++q)
                                u[q] += sums[static_cast<std::size_t>(q) * n + c];
                        }
                if (field.solid[o] || fluid == 0) {
                    field.solid[o] = 1;
                    field.velocity[o] = glm::vec3(0.0f);
                    field.pressure[o] = 0.0f;
                    continue;
                }
                const double inv = 1.0 / fluid;
                field.velocity[o] = glm::vec3(static_cast<float>(u[0] * inv), static_cast<float>(u[1] * inv),
                                              static_cast<float>(u[2] * inv)) *
                                    toVelocity;
                field.pressure[o] = units.kinematicPressure(static_cast<float>(u[3] * inv) / static_cast<float>(samples));
            }
}

} // namespace solvers::lbm
