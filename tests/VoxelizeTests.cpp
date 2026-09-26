// The solid mask used for both the output grid and the LBM lattice.

#include "Check.h"

#include "core/TunnelDomain.h"
#include "core/Voxelize.h"

#include <cmath>

TEST(maskMatchesFieldVoxelisation)
{
    const core::SurfaceMesh sphere = core::makeSphereMesh({0.0f, 0.0f, 0.5f}, 0.5f);
    core::SimulationParams params;
    params.gridCellsX = 160; // 25 cells across the sphere
    core::FlowField f = core::makeFieldForDomain(core::makeTunnelDomain(sphere.bounds(), params));
    for (auto& v : f.velocity)
        v = glm::vec3(1.0f);
    core::voxelizeSolid(sphere, f);
    const auto mask = core::voxelizeSolidMask(sphere, f.origin, f.spacing, f.dims);
    CHECK(mask == f.solid);

    std::size_t solid = 0;
    for (std::size_t c = 0; c < f.cellCount(); ++c) {
        solid += f.solid[c];
        if (f.solid[c])
            CHECK(f.velocity[c] == glm::vec3(0.0f));
    }
    // Volume within a few per cent of the sphere's (majority-vote staircase).
    const double volume = static_cast<double>(solid) * std::pow(f.spacing.x, 3);
    CHECK_NEAR(volume / (4.0 / 3.0 * M_PI * 0.125), 1.0, 0.08);
}

TEST(maskHandlesEmptyInput)
{
    CHECK(core::voxelizeSolidMask(core::SurfaceMesh{}, glm::vec3(0.0f), glm::vec3(1.0f), {2, 2, 2}) ==
          std::vector<std::uint8_t>(8, 0));
    CHECK(core::voxelizeSolidMask(core::makeSphereMesh({0, 0, 0}, 1.0f), glm::vec3(0.0f), glm::vec3(1.0f), {0, 3, 3}).empty());
}
