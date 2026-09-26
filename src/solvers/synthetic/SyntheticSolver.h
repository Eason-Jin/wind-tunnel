#pragma once

#include "core/ISolver.h"
#include "core/TunnelDomain.h"

namespace solvers {

// Instant analytic "solver": inviscid potential flow around a sphere enclosing
// the body's cross-section, with Bernoulli pressure. Not physically accurate
// for arbitrary bodies, but gives the renderer plausible data (stagnation
// point, acceleration over the top, symmetric flow) without running CFD.
class SyntheticSolver : public core::ISolver {
public:
    std::string name() const override { return "Synthetic (potential flow)"; }
    void setup(const core::SurfaceMesh& body, const core::SimulationParams& params) override;
    void run(const core::ProgressFn& progress, const std::atomic<bool>& cancel) override;
    core::FlowField result() const override { return field_; }

private:
    core::SurfaceMesh body_;
    core::SimulationParams params_;
    core::FlowField field_;
};

// The same analytic field, built directly (used by the app's --field synthetic).
core::FlowField makeSyntheticField(const core::SurfaceMesh& body, const core::SimulationParams& params);

} // namespace solvers
