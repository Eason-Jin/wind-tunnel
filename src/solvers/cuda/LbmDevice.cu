// CUDA implementation of GpuLattice. The per-cell physics lives in
// LbmLattice.h; the kernels here only map one thread to one cell.

#include "solvers/cuda/LbmDevice.h"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace solvers::lbm {

namespace {

void check(cudaError_t err, const char* what)
{
    if (err != cudaSuccess)
        throw std::runtime_error(std::string("CUDA: ") + what + " failed: " + cudaGetErrorString(err));
}

std::string megabytes(std::size_t bytes)
{
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.0f MB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    return buf;
}

constexpr int kBlock = 256;

unsigned blocksFor(std::size_t n) { return static_cast<unsigned>((n + kBlock - 1) / kBlock); }

__global__ void initialiseKernel(LatticeParams p, float* f, const std::uint8_t* flags)
{
    const std::size_t c = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (c < p.cells())
        initialiseCell(p, f, flags, c);
}

// One thread per cell; blocks run along x, the grid's y and z are the
// lattice rows, so no index division is needed.
constexpr int kRowBlock = 128;

__global__ void __launch_bounds__(kRowBlock) stepKernel(LatticeParams p, const float* __restrict__ fIn, float* __restrict__ fOut,
                                                        const std::uint8_t* __restrict__ flags, float* sums)
{
    const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (x >= p.nx)
        return;
    updateCell(p, fIn, fOut, flags, sums, x, static_cast<int>(blockIdx.y), static_cast<int>(blockIdx.z));
}

// Largest fluid speed (as float bits: non-negative floats order like their
// bit patterns) and a count of broken cells.
__global__ void healthKernel(LatticeParams p, const float* __restrict__ f, const std::uint8_t* __restrict__ flags,
                             unsigned* maxSpeedBits, unsigned long long* bad)
{
    const std::size_t c = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    float speed = 0.0f;
    bool broken = false;
    if (c < p.cells() && !(flags[c] & kSolid)) {
        float g[kQ];
        for (int k = 0; k < kQ; ++k)
            g[k] = f[static_cast<std::size_t>(k) * p.cells() + c];
        const Moments m = moments(g);
        speed = sqrtf(m.ux * m.ux + m.uy * m.uy + m.uz * m.uz);
        broken = !isfinite(speed) || !isfinite(m.rho) || !(m.rho > 0.0f);
        if (broken)
            speed = 0.0f;
    }
    for (int offset = 16; offset > 0; offset /= 2)
        speed = fmaxf(speed, __shfl_down_sync(0xffffffffu, speed, offset));
    const unsigned brokenMask = __ballot_sync(0xffffffffu, broken);
    if ((threadIdx.x & 31) == 0) {
        atomicMax(maxSpeedBits, __float_as_uint(speed));
        if (brokenMask)
            atomicAdd(bad, static_cast<unsigned long long>(__popc(brokenMask)));
    }
}

__global__ void momentsKernel(LatticeParams p, const float* __restrict__ f, const std::uint8_t* __restrict__ flags,
                              float* __restrict__ out)
{
    const std::size_t c = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    const std::size_t n = p.cells();
    if (c >= n)
        return;
    float u[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (!(flags[c] & kSolid)) {
        float g[kQ];
        for (int k = 0; k < kQ; ++k)
            g[k] = f[static_cast<std::size_t>(k) * n + c];
        const Moments m = moments(g);
        u[0] = m.ux;
        u[1] = m.uy;
        u[2] = m.uz;
        u[3] = m.rho - 1.0f;
    }
    for (int q = 0; q < 4; ++q)
        out[static_cast<std::size_t>(q) * n + c] += u[q];
}

} // namespace

bool deviceAvailable()
{
    static const bool available = [] {
        int count = 0;
        return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
    }();
    return available;
}

DeviceInfo deviceInfo()
{
    if (!deviceAvailable())
        throw std::runtime_error("CUDA: no usable GPU found");
    int device = 0;
    check(cudaGetDevice(&device), "cudaGetDevice");
    cudaDeviceProp prop{};
    check(cudaGetDeviceProperties(&prop, device), "cudaGetDeviceProperties");
    DeviceInfo info;
    info.name = prop.name;
    check(cudaMemGetInfo(&info.freeBytes, &info.totalBytes), "cudaMemGetInfo");
    return info;
}

struct GpuLattice::Impl {
    LatticeParams params;
    float* fA = nullptr; // current post-collision populations
    float* fB = nullptr;
    std::uint8_t* flags = nullptr;
    float* sums = nullptr;
    float* moments = nullptr; // snapshot sums, allocated on first use
    int snapshotSamples = 0;
    unsigned* maxSpeedBits = nullptr;
    unsigned long long* bad = nullptr;
    int step = 0;
    int samples = 0;
    std::size_t bytes = 0;

    ~Impl()
    {
        cudaFree(fA);
        cudaFree(fB);
        cudaFree(flags);
        cudaFree(sums);
        cudaFree(moments);
        cudaFree(maxSpeedBits);
        cudaFree(bad);
    }
};

GpuLattice::GpuLattice(const LatticeParams& params, const std::vector<std::uint8_t>& flags) : impl_(std::make_unique<Impl>())
{
    const std::size_t n = params.cells();
    if (n == 0 || flags.size() != n)
        throw std::runtime_error("LBM: flags do not match the lattice size");
    const DeviceInfo info = deviceInfo();
    const std::size_t populations = n * kQ * sizeof(float);
    const std::size_t need = 2 * populations + n * sizeof(std::uint8_t) + 4 * n * sizeof(float);
    // Leave some headroom for the driver and the app's own rendering.
    const std::size_t reserve = 160u << 20;
    if (need + reserve > info.freeBytes)
        throw std::runtime_error("LBM: the " + std::to_string(params.nx) + "x" + std::to_string(params.ny) + "x" +
                                 std::to_string(params.nz) + " lattice needs " + megabytes(need) + " of GPU memory but only " +
                                 megabytes(info.freeBytes) + " is free on " + info.name + " (lower the quality or grid size)");

    Impl& d = *impl_;
    d.params = params;
    d.bytes = need;
    check(cudaMalloc(&d.fA, populations), "allocating populations");
    check(cudaMalloc(&d.fB, populations), "allocating populations");
    check(cudaMalloc(&d.flags, n), "allocating flags");
    check(cudaMalloc(&d.sums, 4 * n * sizeof(float)), "allocating averages");
    check(cudaMalloc(&d.maxSpeedBits, sizeof(unsigned)), "allocating reduction");
    check(cudaMalloc(&d.bad, sizeof(unsigned long long)), "allocating reduction");
    check(cudaMemcpy(d.flags, flags.data(), n, cudaMemcpyHostToDevice), "uploading flags");
    check(cudaMemset(d.sums, 0, 4 * n * sizeof(float)), "clearing averages");
    initialiseKernel<<<blocksFor(n), kBlock>>>(params, d.fA, d.flags);
    check(cudaGetLastError(), "initialising the lattice");
    check(cudaMemcpy(d.fB, d.fA, populations, cudaMemcpyDeviceToDevice), "initialising the lattice");
    check(cudaDeviceSynchronize(), "initialising the lattice");
}

GpuLattice::~GpuLattice() = default;

void GpuLattice::advance(int count, int accumulateEvery)
{
    Impl& d = *impl_;
    const dim3 rows((d.params.nx + kRowBlock - 1) / kRowBlock, d.params.ny, d.params.nz);
    for (int i = 0; i < count; ++i) {
        const bool sample = accumulateEvery > 0 && d.step % accumulateEvery == 0;
        stepKernel<<<rows, kRowBlock>>>(d.params, d.fA, d.fB, d.flags, sample ? d.sums : nullptr);
        std::swap(d.fA, d.fB);
        ++d.step;
        d.samples += sample ? 1 : 0;
    }
    check(cudaGetLastError(), "launching the LBM step");
    check(cudaDeviceSynchronize(), "running the LBM step");
}

int GpuLattice::step() const { return impl_->step; }
int GpuLattice::samples() const { return impl_->samples; }
std::size_t GpuLattice::deviceBytes() const { return impl_->bytes; }

LatticeHealth GpuLattice::health()
{
    Impl& d = *impl_;
    check(cudaMemset(d.maxSpeedBits, 0, sizeof(unsigned)), "health check");
    check(cudaMemset(d.bad, 0, sizeof(unsigned long long)), "health check");
    healthKernel<<<blocksFor(d.params.cells()), kBlock>>>(d.params, d.fA, d.flags, d.maxSpeedBits, d.bad);
    check(cudaGetLastError(), "health check");
    unsigned bits = 0;
    unsigned long long bad = 0;
    check(cudaMemcpy(&bits, d.maxSpeedBits, sizeof bits, cudaMemcpyDeviceToHost), "health check");
    check(cudaMemcpy(&bad, d.bad, sizeof bad, cudaMemcpyDeviceToHost), "health check");
    LatticeHealth h;
    std::memcpy(&h.maxSpeed, &bits, sizeof bits);
    h.badCells = static_cast<std::size_t>(bad);
    return h;
}

void GpuLattice::readSums(std::vector<float>& out)
{
    const std::size_t n = impl_->params.cells();
    out.resize(4 * n);
    check(cudaMemcpy(out.data(), impl_->sums, 4 * n * sizeof(float), cudaMemcpyDeviceToHost), "reading averages");
}

void GpuLattice::addSnapshotSample()
{
    Impl& d = *impl_;
    const std::size_t n = d.params.cells();
    if (!d.moments) {
        check(cudaMalloc(&d.moments, 4 * n * sizeof(float)), "allocating the snapshot buffer");
        check(cudaMemset(d.moments, 0, 4 * n * sizeof(float)), "clearing the snapshot buffer");
        d.bytes += 4 * n * sizeof(float);
    }
    momentsKernel<<<blocksFor(n), kBlock>>>(d.params, d.fA, d.flags, d.moments);
    check(cudaGetLastError(), "sampling a snapshot");
    ++d.snapshotSamples;
}

int GpuLattice::readSnapshot(std::vector<float>& out)
{
    Impl& d = *impl_;
    const std::size_t n = d.params.cells();
    out.assign(4 * n, 0.0f);
    const int samples = d.snapshotSamples;
    if (d.moments) {
        check(cudaMemcpy(out.data(), d.moments, 4 * n * sizeof(float), cudaMemcpyDeviceToHost), "reading a snapshot");
        check(cudaMemset(d.moments, 0, 4 * n * sizeof(float)), "clearing the snapshot buffer");
    }
    d.snapshotSamples = 0;
    return samples;
}

void GpuLattice::readPopulations(std::vector<float>& out)
{
    const std::size_t n = impl_->params.cells();
    out.resize(kQ * n);
    check(cudaMemcpy(out.data(), impl_->fA, kQ * n * sizeof(float), cudaMemcpyDeviceToHost), "reading populations");
}

} // namespace solvers::lbm
