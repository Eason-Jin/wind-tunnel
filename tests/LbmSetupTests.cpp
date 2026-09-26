// Lattice units, run planning, boundary flags and resampling onto the output grid.

#include "Check.h"

#include "core/SurfaceMesh.h"
#include "core/TunnelDomain.h"
#include "solvers/cuda/LbmSetup.h"

#include <cmath>

using namespace solvers::lbm;

TEST(latticeUnitsConvertBothWays)
{
    const LatticeUnits u = makeLatticeUnits(0.01f, 20.0f, 1.5e-5f, 0.08f);
    CHECK_NEAR(u.dt, 0.01 * 0.08 / 20.0, 1e-11);
    CHECK_NEAR(u.speedScale(), 250.0, 1e-3);
    CHECK_NEAR(u.velocity(0.08f), 20.0, 1e-4);
    CHECK_NEAR(u.nuLattice, 1.5e-5 * u.dt / (0.01 * 0.01), 1e-10);
    CHECK_NEAR(u.tau0, 0.5 + 3.0 * u.nuLattice, 1e-7);
    // Stagnation: rho - 1 = 0.5 u^2 / cs^2 must come out as 0.5 U^2.
    CHECK_NEAR(u.kinematicPressure(1.5f * 0.08f * 0.08f), 0.5 * 20.0 * 20.0, 1e-2);
    CHECK_NEAR(u.kinematicPressure(0.0f), 0.0, 0.0);

    CHECK_THROWS(makeLatticeUnits(0.0f, 20.0f, 1.5e-5f));
    CHECK_THROWS(makeLatticeUnits(0.01f, -1.0f, 1.5e-5f));
    CHECK_THROWS(makeLatticeUnits(0.01f, 20.0f, 0.0f));
}

TEST(planMatchesTheOutputDomain)
{
    core::Bounds body{{-0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 1.0f}};
    core::SimulationParams params;
    params.gridCellsX = 40;
    params.lbmRefine = 2;
    params.lbmFlowThroughs = 2.0f;
    params.inletSpeed = 25.0f;
    params.groundPlane = true;
    const LbmPlan plan = makeLbmPlan(body, params);
    const core::TunnelDomain domain = core::makeTunnelDomain(body, params);

    CHECK(plan.domain.cells == domain.cells);
    CHECK(plan.lattice == domain.cells * 2);
    CHECK_NEAR(plan.units.dx, domain.cellSize / 2.0f, 1e-7);
    // Lattice cell 0 sits half a lattice cell inside the tunnel corner.
    CHECK_NEAR(plan.latticeOrigin.x, domain.box.min.x + 0.5f * plan.units.dx, 1e-6);
    CHECK_NEAR(plan.latticeOrigin.z, domain.box.min.z + 0.5f * plan.units.dx, 1e-6);
    CHECK(plan.movingFloor);
    CHECK(plan.steps == static_cast<int>(std::ceil(2.0f * plan.lattice.x / kLatticeInletSpeed)));
    CHECK(plan.averageFrom > 0 && plan.averageFrom < plan.steps);
    CHECK(plan.deviceBytes == latticeDeviceBytes(plan.cells()));
    CHECK(latticeDeviceBytes(1000) == 1000u * (2 * 19 * 4 + 1 + 16));

    const LatticeParams lp = plan.latticeParams();
    CHECK(lp.nx == plan.lattice.x && lp.ny == plan.lattice.y && lp.nz == plan.lattice.z);
    CHECK_NEAR(lp.uInlet, kLatticeInletSpeed, 0.0);
    CHECK_NEAR(lp.tau0, plan.units.tau0, 0.0);
    CHECK(lp.movingFloor);

    params.groundPlane = false;
    params.lbmRefine = 9; // clamped
    const LbmPlan raised = makeLbmPlan(body, params);
    CHECK(!raised.movingFloor);
    CHECK(raised.refine == 4);
}

TEST(flagsMarkSolidsFacesAndNeighbours)
{
    const glm::ivec3 d{6, 5, 5};
    std::vector<std::uint8_t> solid(static_cast<std::size_t>(d.x * d.y * d.z), 0);
    auto at = [&](int x, int y, int z) { return static_cast<std::size_t>(x + d.x * (y + d.y * z)); };
    solid[at(3, 2, 2)] = 1;
    const auto flags = buildFlags(solid, d);
    CHECK(flags[at(3, 2, 2)] == kSolid);
    CHECK(flags[at(0, 2, 2)] == kNearBoundary);  // inlet face
    CHECK(flags[at(5, 2, 2)] == kNearBoundary);  // outlet face
    CHECK(flags[at(2, 0, 2)] == kNearBoundary);  // side
    CHECK(flags[at(2, 2, 4)] == kNearBoundary);  // top
    CHECK(flags[at(2, 1, 2)] == kNearBoundary);  // diagonal neighbour of the solid
    CHECK(flags[at(4, 3, 3)] == kFluid);         // (1,1,1) away: not a D3Q19 link
    CHECK(flags[at(1, 2, 2)] == kFluid);
    CHECK_THROWS(buildFlags(std::vector<std::uint8_t>(3, 0), d));
}

TEST(resampleAveragesFluidCellsOnly)
{
    // Lattice 4x2x2 = two output cells (refine 2) of 8 lattice cells each.
    const glm::ivec3 lattice{4, 2, 2};
    const std::size_t n = 16;
    std::vector<std::uint8_t> flags(n, kFluid);
    std::vector<float> sums(4 * n, 0.0f);
    const int samples = 10;
    const LatticeUnits units = makeLatticeUnits(0.1f, 20.0f, 1.5e-5f, 0.08f); // 250 m/s per lattice unit
    auto at = [&](int x, int y, int z) { return static_cast<std::size_t>(x + 4 * (y + 2 * z)); };
    for (int z = 0; z < 2; ++z)
        for (int y = 0; y < 2; ++y)
            for (int x = 0; x < 4; ++x) {
                const std::size_t c = at(x, y, z);
                sums[c] = samples * 0.08f;              // ux
                sums[n + c] = samples * 0.01f * x;      // uy varies along x
                sums[2 * n + c] = 0.0f;
                sums[3 * n + c] = samples * 0.003f;     // rho - 1
            }
    // One lattice cell of output cell 0 is solid and holds garbage: ignored.
    flags[at(0, 0, 0)] = kSolid;
    sums[at(0, 0, 0)] = 1e6f;

    core::FlowField f;
    f.origin = glm::vec3(0.0f);
    f.spacing = glm::vec3(0.2f);
    f.resize({2, 1, 1});
    resampleMeans(sums, samples, flags, lattice, 2, units, f);
    CHECK(!f.solid[0] && !f.solid[1]);
    CHECK_NEAR(f.velocity[0].x, 20.0, 1e-3);
    CHECK_NEAR(f.velocity[1].x, 20.0, 1e-3);
    // Output cell 0 averages x = 0 (3 fluid cells) and x = 1 (4 cells): uy = 0.01 * 4/7.
    CHECK_NEAR(f.velocity[0].y, 0.01 * 4.0 / 7.0 * 250.0, 1e-3);
    CHECK_NEAR(f.velocity[1].y, 0.025 * 250.0, 1e-3);
    CHECK_NEAR(f.pressure[1], units.kinematicPressure(0.003f), 1e-3);

    // A fully solid block, and a block the output mask already calls solid.
    for (int z = 0; z < 2; ++z)
        for (int y = 0; y < 2; ++y)
            for (int x = 2; x < 4; ++x)
                flags[at(x, y, z)] = kSolid;
    f.resize({2, 1, 1});
    f.solid[0] = 1;
    resampleMeans(sums, samples, flags, lattice, 2, units, f);
    CHECK(f.solid[0] && f.solid[1]);
    CHECK_NEAR(f.velocity[0].x, 0.0, 0.0);
    CHECK_NEAR(f.velocity[1].x, 0.0, 0.0);
    CHECK_NEAR(f.pressure[1], 0.0, 0.0);

    CHECK_THROWS(resampleMeans(sums, 0, flags, lattice, 2, units, f));
    CHECK_THROWS(resampleMeans(sums, samples, flags, lattice, 3, units, f));
    CHECK_THROWS(resampleMeans(std::vector<float>(3), samples, flags, lattice, 2, units, f));
}

TEST(floorGapsCloseOnlyNearTheFloor)
{
    const glm::ivec3 d{3, 1, 6};
    std::vector<std::uint8_t> solid(18, 0);
    auto at = [&](int x, int z) { return static_cast<std::size_t>(x + 3 * z); };
    solid[at(0, 1)] = 1; // one cell above the floor: gap filled
    solid[at(1, 3)] = 1; // three cells up: left alone
    solid[at(2, 0)] = 1; // already on the floor
    closeFloorGaps(solid, d, 1);
    CHECK(solid[at(0, 0)] == 1);
    CHECK(solid[at(1, 0)] == 0 && solid[at(1, 1)] == 0 && solid[at(1, 2)] == 0);
    CHECK(solid[at(2, 0)] == 1 && solid[at(2, 1)] == 0);
    closeFloorGaps(solid, d, 3);
    CHECK(solid[at(1, 0)] == 1 && solid[at(1, 2)] == 1);
    CHECK_THROWS(closeFloorGaps(solid, {2, 2, 2}, 1));
}

TEST(latticeFlagsVoxeliseTheBody)
{
    // A sphere on the floor: solid cells in the middle, a closed contact
    // patch on a moving floor, none of it when the floor is slip.
    const core::SurfaceMesh sphere = core::makeSphereMesh({0.0f, 0.0f, 0.5f}, 0.5f);
    core::SimulationParams params;
    params.gridCellsX = 24;
    params.lbmRefine = 2;
    LbmPlan plan = makeLbmPlan(sphere.bounds(), params);
    const auto moving = latticeFlags(sphere, plan);
    CHECK(moving.size() == plan.cells());
    auto cellAt = [&](glm::vec3 p) {
        const glm::ivec3 i = glm::ivec3(glm::floor((p - plan.latticeOrigin) / plan.units.dx + 0.5f));
        return static_cast<std::size_t>(i.x + plan.lattice.x * (i.y + plan.lattice.y * i.z));
    };
    CHECK(moving[cellAt({0.0f, 0.0f, 0.5f})] == kSolid);
    CHECK(moving[cellAt({-1.0f, 0.0f, 0.5f})] == kFluid);
    std::size_t floorSolidMoving = 0, floorSolidSlip = 0;
    plan.movingFloor = false;
    const auto slip = latticeFlags(sphere, plan);
    for (std::size_t c = 0; c < static_cast<std::size_t>(plan.lattice.x * plan.lattice.y); ++c) {
        floorSolidMoving += moving[c] == kSolid;
        floorSolidSlip += slip[c] == kSolid;
    }
    CHECK(floorSolidMoving > floorSolidSlip);
}
