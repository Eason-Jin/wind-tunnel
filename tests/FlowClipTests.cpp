// Unsteady snapshots: packing, rescaling and how the GPU run schedules them.

#include "Check.h"

#include "core/FlowClip.h"
#include "core/TunnelDomain.h"
#include "solvers/cuda/LbmSetup.h"

TEST(clipFramesRoundTripAndRescale)
{
    core::FlowClip clip;
    clip.dims = {2, 1, 2};
    clip.freestreamSpeed = 20.0f;
    const std::vector<glm::vec3> u = {{20.0f, 0.0f, 0.0f}, {-3.5f, 1.25f, 0.0f}, {0.0f, 0.0f, 0.0f}, {12.0f, -7.0f, 2.0f}};
    const std::vector<float> p = {0.0f, -150.0f, 210.5f, 3.0f};
    clip.addFrame(u, p);
    CHECK(clip.frameCount() == 1);
    CHECK(clip.bytes() == 4 * sizeof(std::uint64_t));

    std::vector<glm::vec4> out;
    clip.decodeFrame(0, 20.0f, out);
    CHECK(out.size() == 4u);
    for (std::size_t c = 0; c < out.size(); ++c) {
        CHECK_NEAR(out[c].x, u[c].x, 0.02); // half floats: ~3 significant digits
        CHECK_NEAR(out[c].y, u[c].y, 0.02);
        CHECK_NEAR(out[c].w, p[c], 0.2);
    }
    // At twice the speed: velocities double, pressures quadruple.
    clip.decodeFrame(0, 40.0f, out);
    CHECK_NEAR(out[3].x, 24.0, 0.05);
    CHECK_NEAR(out[1].w, -600.0, 1.0);

    bool threw = false;
    try {
        clip.addFrame({glm::vec3(0.0f)}, {0.0f}); // wrong size
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(clipIsScheduledInTheDevelopedFlow)
{
    using namespace solvers::lbm;
    core::Bounds body{{-0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 1.0f}};
    core::SimulationParams params;
    params.gridCellsX = 64;
    params.lbmRefine = 2;
    const LbmPlan plan = makeLbmPlan(body, params);
    const std::size_t outputCells =
        static_cast<std::size_t>(plan.domain.cells.x) * plan.domain.cells.y * plan.domain.cells.z;

    CHECK(plan.clipFrames >= kClipMinFrames && plan.clipFrames <= kClipMaxFrames);
    CHECK(plan.clipFrames * 8 * outputCells <= kClipBudgetBytes);
    // Frames end on the last step, start after the flow has developed and
    // their smoothing windows don't overlap.
    CHECK(plan.clipFrom + (plan.clipFrames - 1) * plan.clipEvery == plan.steps);
    CHECK(plan.clipFrom - plan.clipSmoothing >= plan.averageFrom);
    CHECK(plan.clipSmoothing >= 1 && plan.clipSmoothing <= plan.clipEvery);

    // A grid too big for even the minimum clip records none.
    params.gridCellsX = 2000;
    const LbmPlan huge = makeLbmPlan(body, params);
    CHECK(huge.clipFrames == 0);
}
