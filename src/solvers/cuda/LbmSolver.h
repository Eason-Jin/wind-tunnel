#pragma once

#include "core/ISolver.h"

namespace solvers {

// GPU lattice-Boltzmann backend (CUDA). Voxelises the body onto a lattice
// params.lbmRefine times finer than the output grid, runs a D3Q19
// regularised-BGK large-eddy simulation (Smagorinsky) for
// params.lbmFlowThroughs flow-through times from an impulsive start, and
// returns the time average of the latter 60% of the run, block-averaged onto
// the same output grid as the other solvers (core::makeTunnelDomain).
//
// Boundaries match the OpenFOAM case: velocity inlet, fixed-pressure outlet,
// slip sides and top, and a floor that is a moving no-slip ground
// (params.groundPlane) or slip. The body is a no-slip staircase (halfway
// bounce-back) at lattice resolution.
class LbmSolver : public core::ISolver {
public:
    std::string name() const override { return "GPU lattice-Boltzmann (CUDA)"; }
    void setup(const core::SurfaceMesh& body, const core::SimulationParams& params) override;
    void run(const core::ProgressFn& progress, const std::atomic<bool>& cancel) override;
    core::FlowField result() const override { return field_; }

    // True when built with CUDA and a CUDA-capable GPU is present.
    static bool available();

private:
    core::SurfaceMesh body_;
    core::SimulationParams params_;
    core::FlowField field_;
};

} // namespace solvers
