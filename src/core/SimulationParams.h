#pragma once

#include <filesystem>

namespace core {

// Solver-independent description of a wind tunnel run. Flow is along +x,
// +z is up. The body mesh handed to a solver is already in metres (the app
// applies the STL unit scale before calling setup()).
struct SimulationParams {
    float inletSpeed = 20.0f;           // m/s along +x
    float kinematicViscosity = 1.5e-5f; // m^2/s (air at ~20 C)

    // Tunnel size as multiples of the body's largest dimension L.
    float upstream = 1.5f;   // inlet distance ahead of the body
    float downstream = 4.0f; // outlet distance behind the body
    float side = 1.5f;       // clearance on +-y and above the body
    bool groundPlane = true; // body sits on the tunnel floor (z = body min z) if true

    int gridCellsX = 128;    // resolution of the output FlowField along x (y, z derived to keep cells cubic)
    int iterations = 400;    // steady solver iterations
    int refinementLevel = 3; // surface refinement level for meshing solvers
    int processors = 1;      // parallel ranks for solvers that support MPI

    std::filesystem::path workDir = "cases/run"; // scratch directory for file-based solvers
};

} // namespace core
