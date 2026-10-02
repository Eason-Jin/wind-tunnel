#include "solvers/cuda/LbmSolver.h"

#include "core/TunnelDomain.h"
#include "core/Voxelize.h"
#include "solvers/cuda/LbmDevice.h"
#include "solvers/cuda/LbmSetup.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace solvers {

namespace {

double seconds(std::chrono::steady_clock::time_point since)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

std::string dimsText(const glm::ivec3& d)
{
    return std::to_string(d.x) + "x" + std::to_string(d.y) + "x" + std::to_string(d.z);
}

} // namespace

bool LbmSolver::available() { return lbm::deviceAvailable(); }

void LbmSolver::setup(const core::SurfaceMesh& body, const core::SimulationParams& params)
{
    if (body.empty())
        throw std::runtime_error("LBM: body mesh is empty");
    if (params.inletSpeed <= 0.0f || params.kinematicViscosity <= 0.0f)
        throw std::runtime_error("LBM: inlet speed and viscosity must be positive");
    body_ = body;
    params_ = params;
    field_ = core::FlowField{};
}

void LbmSolver::run(const core::ProgressFn& progress, const std::atomic<bool>& cancel)
{
    field_ = core::FlowField{};
    if (body_.empty())
        throw std::runtime_error("LBM: setup() was not called");
    if (!lbm::deviceAvailable())
        throw std::runtime_error("LBM: no CUDA GPU available (or this build has no CUDA support)");

    auto report = [&](const std::string& stage, float fraction, const std::string& message) {
        if (progress)
            progress({stage, std::clamp(fraction, 0.0f, 1.0f), message});
    };
    auto cancelled = [&] {
        if (!cancel)
            return false;
        report("Cancelled", 0.0f, "LBM run cancelled");
        return true;
    };

    const auto t0 = std::chrono::steady_clock::now();
    const lbm::LbmPlan plan = lbm::makeLbmPlan(body_.bounds(), params_);
    const lbm::LatticeParams lp = plan.latticeParams();

    // --- Lattice -----------------------------------------------------------
    report("Meshing", 0.0f, "Voxelising the body on a " + dimsText(plan.lattice) + " lattice");
    const std::vector<std::uint8_t> flags = lbm::latticeFlags(body_, plan);
    if (cancelled())
        return;

    report("Meshing", 0.02f, "Allocating " + std::to_string(plan.deviceBytes >> 20) + " MB on the GPU");
    lbm::GpuLattice lattice(lp, flags);
    const double setupTime = seconds(t0);

    // Output grid template (solid mask), shared by the clip frames and the mean.
    core::FlowField base = core::makeFieldForDomain(plan.domain);
    base.freestreamSpeed = params_.inletSpeed;
    core::voxelizeSolid(body_, base);

    // Clip of instantaneous snapshots for playback (the result is the mean).
    auto clip = std::make_shared<core::FlowClip>();
    clip->dims = base.dims;
    clip->freestreamSpeed = params_.inletSpeed;
    clip->frameSeconds = static_cast<float>(plan.clipEvery) * plan.units.dt;
    int clipFrames = plan.clipFrames;
    std::vector<float> snapshot;
    auto captureFrame = [&] {
        try {
            const int samples = lattice.readSnapshot(snapshot);
            core::FlowField frame = base;
            lbm::resampleMeans(snapshot, samples, flags, plan.lattice, plan.refine, plan.units, frame);
            clip->addFrame(frame.velocity, frame.pressure);
        } catch (const std::exception&) {
            clipFrames = 0; // e.g. no GPU memory left for the snapshot buffer: keep the run, drop the clip
            clip->frames.clear();
        }
    };

    // --- Time stepping -----------------------------------------------------
    constexpr float f0 = 0.03f, f1 = 0.97f;
    constexpr int kChunk = 50;          // steps between cancel checks / progress
    constexpr int kHealthEvery = 1000;  // steps between divergence checks
    const auto solveStart = std::chrono::steady_clock::now();
    auto lastReport = solveStart - std::chrono::seconds(1);
    int nextHealth = kHealthEvery;
    float lastMaxSpeed = plan.units.uLattice;
    while (lattice.step() < plan.steps) {
        if (cancelled())
            return;
        const int step = lattice.step();
        // Never run a chunk across the start of the averaging window.
        int count = std::min(kChunk, plan.steps - step);
        if (step < plan.averageFrom)
            count = std::min(count, plan.averageFrom - step);
        // ...nor across a clip frame.
        // Each frame averages the states after steps (windowStart, nextFrame],
        // so those are run one at a time.
        const int nextFrame = plan.clipFrom + clip->frameCount() * plan.clipEvery;
        const int windowStart = nextFrame - plan.clipSmoothing;
        const bool recording = clip->frameCount() < clipFrames;
        if (recording && step >= windowStart && step < nextFrame)
            count = 1;
        else if (recording && step < windowStart)
            count = std::min(count, windowStart - step);
        lattice.advance(count, step >= plan.averageFrom ? plan.averageEvery : 0);
        if (recording && lattice.step() > windowStart && lattice.step() <= nextFrame) {
            try {
                lattice.addSnapshotSample();
            } catch (const std::exception&) {
                clipFrames = 0; // no GPU memory for the snapshot buffer: keep the run, drop the clip
                clip->frames.clear();
            }
        }
        if (recording && clipFrames > 0 && lattice.step() == nextFrame)
            captureFrame();

        if (lattice.step() >= nextHealth || lattice.step() == plan.steps) {
            nextHealth += kHealthEvery;
            const lbm::LatticeHealth h = lattice.health();
            // Speeds near the lattice speed of sound mean the run is blowing up.
            if (h.badCells > 0 || h.maxSpeed > 0.45f)
                throw std::runtime_error("LBM: the simulation became unstable at step " + std::to_string(lattice.step()) +
                                         " (try a different quality or grid size)");
            lastMaxSpeed = h.maxSpeed;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - lastReport > std::chrono::milliseconds(250) || lattice.step() == plan.steps) {
            lastReport = now;
            const double elapsed = seconds(solveStart);
            const double mlups = static_cast<double>(plan.cells()) * lattice.step() / std::max(elapsed, 1e-6) * 1e-6;
            const double remaining = elapsed / lattice.step() * (plan.steps - lattice.step());
            char buf[160];
            std::snprintf(buf, sizeof buf, "Step %d/%d  %s  %.0f MLUPS  peak %.2f U  ~%.0f s left", lattice.step(), plan.steps,
                          clip->frameCount() > 0 ? "recording clip" : lattice.step() > plan.averageFrom ? "averaging" : "developing", mlups,
                          lastMaxSpeed / plan.units.uLattice, remaining);
            report("Solving", f0 + (f1 - f0) * static_cast<float>(lattice.step()) / static_cast<float>(plan.steps), buf);
        }
    }
    const double solveTime = seconds(solveStart);

    // --- Output --------------------------------------------------------------
    report("Mapping", f1, "Averaging onto the output grid");
    std::vector<float> sums;
    lattice.readSums(sums);
    core::FlowField f = std::move(base);
    lbm::resampleMeans(sums, lattice.samples(), flags, plan.lattice, plan.refine, plan.units, f);
    if (clip->frameCount() > 0)
        f.clip = std::move(clip);
    field_ = std::move(f);

    char buf[256];
    std::snprintf(buf, sizeof buf,
                  "LBM: %s lattice, %d steps (%d averaged) in %.0f s (%.0f MLUPS, setup %.1f s), %.0f MB GPU memory, output %s grid, "
                  "%d-frame clip",
                  dimsText(plan.lattice).c_str(), plan.steps, lattice.samples(), solveTime,
                  static_cast<double>(plan.cells()) * plan.steps / std::max(solveTime, 1e-6) * 1e-6, setupTime,
                  static_cast<double>(lattice.deviceBytes()) / (1024.0 * 1024.0), dimsText(field_.dims).c_str(),
                  field_.clip ? field_.clip->frameCount() : 0);
    report("Done", 1.0f, buf);
}

} // namespace solvers
