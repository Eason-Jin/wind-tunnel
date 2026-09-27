// Vortex-core detection used to seed the swirl lines.

#include "Check.h"

#include "core/VortexCores.h"

#include <cmath>

namespace {

// A uniform stream along +x carrying a Lamb-Oseen vortex whose axis is the
// line (y, z) = (y0, z0): swirl speed peaks one core radius out.
core::FlowField streamWithVortex(float y0, float z0, float swirl)
{
    core::FlowField f;
    f.resize({40, 40, 40});
    f.origin = glm::vec3(-1.0f + 0.025f);
    f.spacing = glm::vec3(0.05f);
    f.freestreamSpeed = 10.0f;
    const float rc = 0.15f;
    for (int k = 0; k < 40; ++k)
        for (int j = 0; j < 40; ++j)
            for (int i = 0; i < 40; ++i) {
                const glm::vec3 p = f.cellCentre(i, j, k);
                const float dy = p.y - y0, dz = p.z - z0;
                const float r2 = dy * dy + dz * dz;
                const float vt = r2 > 1e-12f ? swirl * rc * (1.0f - std::exp(-r2 / (rc * rc))) / r2 : 0.0f; // v_theta / r
                f.velocity[f.index(i, j, k)] = glm::vec3(10.0f, -vt * dz, vt * dy);
            }
    return f;
}

} // namespace

TEST(findsTheVortexAxis)
{
    const core::FlowField f = streamWithVortex(0.2f, -0.1f, 8.0f);
    core::VortexCoreOptions options;
    options.charLength = 1.0f;
    options.maxCores = 3;
    options.minSeparation = 0.5f;
    const auto cores = core::findVortexCores(f, options);
    CHECK(!cores.empty());
    if (cores.empty())
        return;
    CHECK_NEAR(cores[0].position.y, 0.2, 0.05);
    CHECK_NEAR(cores[0].position.z, -0.1, 0.05);
    CHECK_NEAR(std::abs(cores[0].axis.x), 1.0, 1e-3); // anticlockwise about +x
    CHECK(cores[0].axis.x > 0.0f);
    // Picks along the same tube are at least minSeparation apart.
    for (std::size_t a = 0; a < cores.size(); ++a)
        for (std::size_t b = a + 1; b < cores.size(); ++b)
            CHECK(glm::length(cores[a].position - cores[b].position) >= options.minSeparation);
}

TEST(ignoresWeakSwirlAndTheBodySkin)
{
    core::VortexCoreOptions options;
    options.minStrength = 1e6f; // far above anything in the field
    CHECK(core::findVortexCores(streamWithVortex(0.0f, 0.0f, 8.0f), options).empty());

    // A solid block over the vortex axis: the wall gap hides everything within it.
    core::FlowField f = streamWithVortex(0.0f, 0.0f, 8.0f);
    f.solid.assign(f.cellCount(), 0);
    for (int k = 0; k < 40; ++k)
        for (int j = 0; j < 40; ++j)
            for (int i = 0; i < 40; ++i)
                if (std::abs(j - 19.5f) < 6 && std::abs(k - 19.5f) < 6)
                    f.solid[f.index(i, j, k)] = 1;
    options.minStrength = 1.0f;
    for (const auto& c : core::findVortexCores(f, options))
        CHECK(std::abs(c.position.y) > 0.4f || std::abs(c.position.z) > 0.4f);

    CHECK(core::findVortexCores(core::FlowField{}, options).empty());
}

TEST(ringIsPerpendicularToTheAxis)
{
    core::VortexCore c;
    c.position = {1.0f, 2.0f, 3.0f};
    c.axis = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f));
    const auto ring = core::ringAround(c, 0.1f, 6);
    CHECK(ring.size() == 6u);
    for (const glm::vec3& p : ring) {
        CHECK_NEAR(glm::length(p - c.position), 0.1, 1e-5);
        CHECK_NEAR(glm::dot(p - c.position, c.axis), 0.0, 1e-5);
    }
    c.axis = {0.0f, 0.0f, 1.0f}; // the other branch of the basis helper
    for (const glm::vec3& p : core::ringAround(c, 0.1f, 4))
        CHECK_NEAR(p.z, 3.0, 1e-5);
    CHECK(core::ringAround(c, 0.1f, 0).empty());
}
