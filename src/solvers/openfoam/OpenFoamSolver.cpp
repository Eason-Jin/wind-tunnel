#include "solvers/openfoam/OpenFoamSolver.h"

#include <stdexcept>

namespace solvers {

// TODO: stub, to be implemented.
void OpenFoamSolver::setup(const core::SurfaceMesh& body, const core::SimulationParams& params)
{
    body_ = body;
    params_ = params;
}

void OpenFoamSolver::run(const core::ProgressFn&, const std::atomic<bool>&)
{
    throw std::runtime_error("OpenFoamSolver not implemented yet");
}

bool OpenFoamSolver::available() { return false; }

core::FlowField loadOpenFoamResult(const std::filesystem::path& workDir)
{
    throw std::runtime_error("loadOpenFoamResult not implemented yet: " + workDir.string());
}

} // namespace solvers
