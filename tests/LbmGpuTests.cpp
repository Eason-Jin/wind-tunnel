// GPU lattice and the full LbmSolver. Skipped when there is no CUDA device
// (or the build has no CUDA).

#include "Check.h"
#include "LbmReference.h"

#include "core/SurfaceMesh.h"
#include "solvers/cuda/LbmDevice.h"
#include "solvers/cuda/LbmSetup.h"
#include "solvers/cuda/LbmSolver.h"

#include <algorithm>
#include <atomic>
#include <cmath>

using namespace solvers::lbm;

namespace {

void requireGpu()
{
    if (!solvers::LbmSolver::available())
        SKIP("no CUDA device (or built without CUDA)");
}

} // namespace

TEST(availabilityIsConsistent)
{
    CHECK(solvers::LbmSolver::available() == deviceAvailable());
    if (!deviceAvailable()) {
        CHECK_THROWS(deviceInfo());
        LatticeParams p;
        p.nx = p.ny = p.nz = 2;
        CHECK_THROWS(GpuLattice(p, std::vector<std::uint8_t>(8, 0)));
    }
}

TEST(gpuMatchesCpuReference)
{
    requireGpu();
    LatticeParams p;
    p.nx = 30;
    p.ny = 14;
    p.nz = 12;
    p.uInlet = 0.08f;
    p.tau0 = 0.50002f;
    p.movingFloor = true;
    // A block standing on the floor, so every boundary rule is exercised.
    std::vector<std::uint8_t> solid(p.cells(), 0);
    for (int z = 0; z <= 4; ++z)
        for (int y = 5; y <= 8; ++y)
            for (int x = 10; x <= 13; ++x)
                solid[p.index(x, y, z)] = 1;
    const auto flags = buildFlags(solid, {p.nx, p.ny, p.nz});

    test::CpuLattice cpu(p, flags);
    GpuLattice gpu(p, flags);
    CHECK(gpu.deviceBytes() == latticeDeviceBytes(p.cells()));
    cpu.advance(40);
    gpu.advance(40);
    cpu.advance(21, 3);
    gpu.advance(21, 3);
    CHECK(gpu.step() == cpu.step);
    CHECK(gpu.samples() == cpu.samples);

    std::vector<float> f, sums;
    gpu.readPopulations(f);
    gpu.readSums(sums);
    float worstF = 0.0f, worstSum = 0.0f;
    for (std::size_t i = 0; i < f.size(); ++i)
        if (!(flags[i % p.cells()] & kSolid))
            worstF = std::max(worstF, std::abs(f[i] - cpu.fA[i]));
    for (std::size_t i = 0; i < sums.size(); ++i)
        worstSum = std::max(worstSum, std::abs(sums[i] - cpu.sums[i]));
    // Same code on both; only FMA contraction and rounding order differ.
    CHECK(worstF < 2e-6f);
    CHECK(worstSum < 2e-5f);

    const LatticeHealth h = gpu.health();
    CHECK(h.badCells == 0);
    CHECK(h.maxSpeed > p.uInlet && h.maxSpeed < 0.3f);
}

TEST(solverProducesWakeBehindSphere)
{
    requireGpu();
    // A 1 m sphere on the floor, coarse: 48 output cells along x, lattice 2x finer.
    const core::SurfaceMesh sphere = core::makeSphereMesh({0.0f, 0.0f, 0.5f}, 0.5f);
    core::SimulationParams params;
    params.inletSpeed = 20.0f;
    params.gridCellsX = 48;
    params.lbmRefine = 2;
    params.lbmFlowThroughs = 1.6f;
    solvers::LbmSolver solver;
    solver.setup(sphere, params);
    std::string lastStage;
    std::atomic<bool> cancel{false};
    solver.run([&](const core::SolverProgress& p) { lastStage = p.stage; }, cancel);
    CHECK(lastStage == "Done");
    const core::FlowField f = solver.result();
    CHECK(!f.empty());
    CHECK_NEAR(f.freestreamSpeed, 20.0, 1e-6);

    bool finite = true;
    std::size_t solid = 0;
    for (std::size_t c = 0; c < f.cellCount(); ++c) {
        finite = finite && std::isfinite(f.velocity[c].x) && std::isfinite(f.pressure[c]);
        solid += f.solid[c];
    }
    CHECK(finite);
    CHECK(solid > 0);

    const float U = params.inletSpeed, q = 0.5f * U * U;
    // Far upstream the stream is undisturbed.
    CHECK_NEAR(f.sampleVelocity({-1.6f, 0.0f, 1.2f}).x / U, 1.0, 0.05);
    // Stagnation point on the nose, suction over the top.
    CHECK(f.samplePressure({-0.56f, 0.0f, 0.5f}) / q > 0.6f);
    CHECK(f.samplePressure({0.0f, 0.0f, 1.07f}) / q < -0.3f);
    // Wake: slow air behind the sphere.
    CHECK(f.sampleVelocity({0.8f, 0.0f, 0.5f}).x / U < 0.6f);
}

TEST(solverCancelsAndValidates)
{
    requireGpu();
    solvers::LbmSolver solver;
    std::atomic<bool> cancel{true};
    CHECK_THROWS(solver.run({}, cancel)); // no setup
    core::SimulationParams params;
    CHECK_THROWS(solver.setup(core::SurfaceMesh{}, params));
    params.inletSpeed = 0.0f;
    CHECK_THROWS(solver.setup(core::makeSphereMesh({0, 0, 0.5f}, 0.5f), params));

    params = core::SimulationParams{};
    params.gridCellsX = 32;
    solver.setup(core::makeSphereMesh({0, 0, 0.5f}, 0.5f), params);
    std::string stage;
    solver.run([&](const core::SolverProgress& p) { stage = p.stage; }, cancel);
    CHECK(stage == "Cancelled");
    CHECK(solver.result().empty());
}
