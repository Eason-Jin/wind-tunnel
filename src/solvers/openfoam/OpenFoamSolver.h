#pragma once

#include "core/ISolver.h"

#include <filesystem>

namespace solvers {

// OpenFOAM (ESI v2406) backend. Runs the OpenFOAM command-line tools as child
// processes inside params.workDir: writes a case around the body, meshes it
// (blockMesh + snappyHexMesh), solves steady incompressible RANS (simpleFoam,
// k-omega SST), then maps the result onto a uniform grid matching
// core::makeTunnelDomain and parses it into a FlowField.
class OpenFoamSolver : public core::ISolver {
public:
    std::string name() const override { return "OpenFOAM (simpleFoam)"; }
    void setup(const core::SurfaceMesh& body, const core::SimulationParams& params) override;
    void run(const core::ProgressFn& progress, const std::atomic<bool>& cancel) override;
    core::FlowField result() const override { return field_; }

    // True if the OpenFOAM tools can be found (sourced environment or the
    // default install at /usr/lib/openfoam/openfoam2406).
    static bool available();

private:
    core::SurfaceMesh body_;
    core::SimulationParams params_;
    core::FlowField field_;
};

// Load the result of a previously completed OpenFoamSolver run from its work
// directory (so the app can reopen results without re-solving).
// Throws std::runtime_error if the directory holds no usable result.
core::FlowField loadOpenFoamResult(const std::filesystem::path& workDir);

} // namespace solvers
