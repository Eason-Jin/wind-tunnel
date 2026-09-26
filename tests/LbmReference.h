#pragma once

// A CPU lattice running exactly the per-cell update the GPU kernel runs
// (solvers/cuda/LbmLattice.h), one cell at a time. Used to test the physics
// without a GPU and as the reference the GPU result must match.

#include "solvers/cuda/LbmLattice.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace test {

struct CpuLattice {
    solvers::lbm::LatticeParams p;
    std::vector<std::uint8_t> flags;
    std::vector<float> fA, fB, sums;
    int step = 0;
    int samples = 0;

    CpuLattice(const solvers::lbm::LatticeParams& params, std::vector<std::uint8_t> cellFlags)
        : p(params), flags(std::move(cellFlags))
    {
        const std::size_t n = p.cells();
        fA.assign(solvers::lbm::kQ * n, 0.0f);
        sums.assign(4 * n, 0.0f);
        for (std::size_t c = 0; c < n; ++c)
            solvers::lbm::initialiseCell(p, fA.data(), flags.data(), c);
        fB = fA;
    }

    // Same sampling rule as GpuLattice::advance().
    void advance(int count, int accumulateEvery = 0)
    {
        for (int i = 0; i < count; ++i) {
            const bool sample = accumulateEvery > 0 && step % accumulateEvery == 0;
            for (int z = 0; z < p.nz; ++z)
                for (int y = 0; y < p.ny; ++y)
                    for (int x = 0; x < p.nx; ++x)
                        solvers::lbm::updateCell(p, fA.data(), fB.data(), flags.data(), sample ? sums.data() : nullptr, x, y, z);
            std::swap(fA, fB);
            ++step;
            samples += sample ? 1 : 0;
        }
    }

    solvers::lbm::Moments at(int x, int y, int z) const
    {
        float f[solvers::lbm::kQ];
        const std::size_t c = p.index(x, y, z);
        for (int k = 0; k < solvers::lbm::kQ; ++k)
            f[k] = fA[static_cast<std::size_t>(k) * p.cells() + c];
        return solvers::lbm::moments(f);
    }
};

} // namespace test
