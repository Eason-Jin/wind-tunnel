// Stand-in for LbmDevice.cu in builds without CUDA: no device, and any
// attempt to build a lattice fails with a clear message.

#if !defined(WT_WITH_CUDA)

#include "solvers/cuda/LbmDevice.h"

#include <stdexcept>

namespace solvers::lbm {

namespace {
[[noreturn]] void unavailable() { throw std::runtime_error("GPU solver unavailable: this build has no CUDA support"); }
} // namespace

bool deviceAvailable() { return false; }
DeviceInfo deviceInfo() { unavailable(); }

struct GpuLattice::Impl {};

GpuLattice::GpuLattice(const LatticeParams&, const std::vector<std::uint8_t>&) { unavailable(); }
GpuLattice::~GpuLattice() = default;
void GpuLattice::advance(int, int) { unavailable(); }
int GpuLattice::step() const { return 0; }
int GpuLattice::samples() const { return 0; }
LatticeHealth GpuLattice::health() { unavailable(); }
void GpuLattice::readSums(std::vector<float>&) { unavailable(); }
void GpuLattice::readPopulations(std::vector<float>&) { unavailable(); }
void GpuLattice::addSnapshotSample() { unavailable(); }
int GpuLattice::readSnapshot(std::vector<float>&) { unavailable(); }
std::size_t GpuLattice::deviceBytes() const { return 0; }

} // namespace solvers::lbm

#endif
