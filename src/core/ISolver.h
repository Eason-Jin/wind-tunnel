#pragma once

#include "core/FlowField.h"
#include "core/SimulationParams.h"
#include "core/SurfaceMesh.h"

#include <atomic>
#include <functional>
#include <string>

namespace core {

struct SolverProgress {
    std::string stage;   // e.g. "Meshing", "Solving"
    float fraction = 0;  // overall progress 0..1 (best estimate)
    std::string message; // latest log line or status text
};

using ProgressFn = std::function<void(const SolverProgress&)>;

// A CFD backend. The app calls setup() then run() on a worker thread, then
// result() once run() returns. Implementations report failure by throwing
// std::runtime_error from setup()/run(), and must return promptly (throwing
// or returning) when `cancel` becomes true.
class ISolver {
public:
    virtual ~ISolver() = default;

    virtual std::string name() const = 0;
    virtual void setup(const SurfaceMesh& body, const SimulationParams& params) = 0;
    virtual void run(const ProgressFn& progress, const std::atomic<bool>& cancel) = 0;
    virtual FlowField result() const = 0;

    // Live solvers (e.g. a future CUDA LBM) can publish intermediate fields.
    virtual bool supportsLive() const { return false; }
};

} // namespace core
