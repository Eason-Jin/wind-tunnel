#pragma once

// GPU side of the lattice-Boltzmann solver. Implemented in LbmDevice.cu when
// the project is built with CUDA (WT_WITH_CUDA); otherwise a stub reports no
// device and refuses to construct a lattice.

#include "solvers/cuda/LbmLattice.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace solvers::lbm {

struct DeviceInfo {
    std::string name;
    std::size_t freeBytes = 0;
    std::size_t totalBytes = 0;
};

// True when built with CUDA and at least one CUDA device can be used.
// Cached after the first call.
bool deviceAvailable();

// Name and memory of the device in use; throws std::runtime_error if none.
DeviceInfo deviceInfo();

// Health of the current state (checked now and then to catch divergence).
struct LatticeHealth {
    float maxSpeed = 0.0f;  // largest fluid speed (lattice units)
    std::size_t badCells = 0; // cells with a NaN/Inf or non-positive density
};

// A lattice resident on the GPU: two population sets (pull streaming from
// one into the other), cell flags and time-average sums.
class GpuLattice {
public:
    // Allocates and initialises the lattice (uniform inlet flow). Throws
    // std::runtime_error when there is no device or not enough memory.
    GpuLattice(const LatticeParams& params, const std::vector<std::uint8_t>& flags);
    ~GpuLattice();
    GpuLattice(const GpuLattice&) = delete;
    GpuLattice& operator=(const GpuLattice&) = delete;

    // Run `count` time steps. When `accumulateEvery` > 0, every step whose
    // index (counted over the lattice's lifetime) is a multiple of it adds its
    // state to the time-average sums. Blocks until the GPU is done.
    void advance(int count, int accumulateEvery = 0);

    int step() const;    // steps run so far
    int samples() const; // states added to the time-average sums

    LatticeHealth health();
    // Copy the sums (ux, uy, uz, rho - 1; structure of arrays) to the host.
    void readSums(std::vector<float>& out);
    // Copy the latest post-collision populations (structure of arrays).
    void readPopulations(std::vector<float>& out);

    std::size_t deviceBytes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace solvers::lbm
