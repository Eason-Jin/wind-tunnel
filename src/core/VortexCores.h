#pragma once

#include "core/FlowField.h"

#include <glm/glm.hpp>

#include <vector>

namespace core {

// A point on the axis of a vortex: where the Q-criterion (rotation minus
// strain) peaks locally. `axis` is the unit vorticity direction there, i.e.
// the direction the vortex tube runs; `strength` is Q normalised by
// (U / L)^2, the same Q* the vortex-surface view uses.
struct VortexCore {
    glm::vec3 position{0.0f};
    glm::vec3 axis{1.0f, 0.0f, 0.0f};
    float strength = 0.0f;
};

struct VortexCoreOptions {
    float charLength = 1.0f;    // L in Q* = Q / (U / L)^2, usually the body length
    float minStrength = 1.0f;   // ignore cores weaker than this Q*
    int maxCores = 8;           // strongest first
    float minSeparation = 0.2f; // metres between chosen cores, so one tube isn't picked many times
    int wallGap = 3;            // cells to skip next to the body: wall shear there is not a vortex
};

// The strongest vortex cores in the field, strongest first.
std::vector<VortexCore> findVortexCores(const FlowField& field, const VortexCoreOptions& options);

// `count` points on a circle of `radius` around the core, perpendicular to its
// axis. Streamlines started here wind around the vortex instead of running
// straight down its middle.
std::vector<glm::vec3> ringAround(const VortexCore& core, float radius, int count);

} // namespace core
