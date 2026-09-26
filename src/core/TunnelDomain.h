#pragma once

#include "core/FlowField.h"
#include "core/SimulationParams.h"
#include "core/SurfaceMesh.h"

namespace core {

// The tunnel box around a body, shared by all solvers so every backend
// produces a FlowField over the same region and grid.
struct TunnelDomain {
    Bounds box;          // tunnel walls (inlet at box.min.x, outlet at box.max.x)
    glm::ivec3 cells{0}; // output grid resolution (cubic cells)
    float cellSize = 0.0f;
};

TunnelDomain makeTunnelDomain(const Bounds& body, const SimulationParams& params);

// An empty (zeroed) FlowField laid out on the domain's output grid.
FlowField makeFieldForDomain(const TunnelDomain& domain);

} // namespace core
