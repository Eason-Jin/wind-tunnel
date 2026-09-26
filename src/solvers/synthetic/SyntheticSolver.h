#pragma once

#include "core/ISolver.h"
#include "core/TunnelDomain.h"

namespace solvers {

// Instant "solver": inviscid potential flow around the actual (voxelised)
// body, solved as Laplace's equation for the velocity potential on the output
// grid, with Bernoulli pressure. It captures stagnation, acceleration and
// flow turning around the real shape, but no separation, wake or vortices;
// use OpenFOAM for those.
class SyntheticSolver : public core::ISolver {
public:
    std::string name() const override { return "Instant preview (potential flow)"; }
    void setup(const core::SurfaceMesh& body, const core::SimulationParams& params) override;
    void run(const core::ProgressFn& progress, const std::atomic<bool>& cancel) override;
    core::FlowField result() const override { return field_; }

private:
    core::SurfaceMesh body_;
    core::SimulationParams params_;
    core::FlowField field_;
};

// The same analytic field, built directly (used by the app's --field synthetic).
// Returns an empty field if `cancel` becomes true part-way.
core::FlowField makeSyntheticField(const core::SurfaceMesh& body, const core::SimulationParams& params,
                                   const std::atomic<bool>* cancel = nullptr, const core::ProgressFn* progress = nullptr);

} // namespace solvers
