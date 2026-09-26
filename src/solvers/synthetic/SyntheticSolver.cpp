#include "solvers/synthetic/SyntheticSolver.h"

#include "core/Voxelize.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <vector>

namespace solvers {

namespace {

// Run fn(begin, end) over [0, n) split across hardware threads.
template <typename Fn>
void parallelFor(std::size_t n, Fn fn)
{
    // Physical cores only: the loops are memory-bound, and SMT siblings add
    // spawn overhead without adding bandwidth.
    const std::size_t threads = std::clamp<std::size_t>(std::thread::hardware_concurrency() / 2, 1, 8);
    std::vector<std::thread> pool;
    for (std::size_t t = 1; t < threads; ++t)
        pool.emplace_back([&, t] { fn(n * t / threads, n * (t + 1) / threads); });
    fn(0, n / threads);
    for (auto& th : pool)
        th.join();
}

// Solve Laplace's equation for the freestream-normalised velocity potential
// psi on the uniform grid: psi = x (freestream) just outside the inlet and
// outlet, zero normal gradient on the tunnel walls and on every face shared
// with a solid cell. The discrete operator (a graph Laplacian with Dirichlet
// ends) is symmetric positive definite, so Jacobi-preconditioned conjugate
// gradients converge in a few hundred iterations.
std::vector<float> solvePotential(const core::FlowField& f, const std::atomic<bool>* cancel, const core::ProgressFn* progress)
{
    const glm::ivec3 d = f.dims;
    const float h = f.spacing.x;
    const std::size_t N = f.cellCount();
    const std::size_t sx = 1, sy = static_cast<std::size_t>(d.x), sz = sy * static_cast<std::size_t>(d.y);

    // Per-cell diagonal (number of active neighbour faces) and right-hand side.
    std::vector<float> diag(N, 0.0f), rhs(N, 0.0f);
    for (int k = 0; k < d.z; ++k)
        for (int j = 0; j < d.y; ++j)
            for (int i = 0; i < d.x; ++i) {
                const std::size_t c = f.index(i, j, k);
                if (f.solid[c])
                    continue;
                float count = 0.0f;
                const float x = f.origin.x + h * static_cast<float>(i);
                if (i == 0) {
                    count += 1.0f;
                    rhs[c] += x - h;
                } else if (!f.solid[c - sx])
                    count += 1.0f;
                if (i == d.x - 1) {
                    count += 1.0f;
                    rhs[c] += x + h;
                } else if (!f.solid[c + sx])
                    count += 1.0f;
                if (j > 0 && !f.solid[c - sy])
                    count += 1.0f;
                if (j < d.y - 1 && !f.solid[c + sy])
                    count += 1.0f;
                if (k > 0 && !f.solid[c - sz])
                    count += 1.0f;
                if (k < d.z - 1 && !f.solid[c + sz])
                    count += 1.0f;
                diag[c] = count;
            }

    // y = A x over fluid cells (solid cells stay 0).
    auto apply = [&](const std::vector<float>& x, std::vector<float>& y) {
        parallelFor(N, [&](std::size_t begin, std::size_t end) {
            for (std::size_t c = begin; c < end; ++c) {
                if (diag[c] == 0.0f) {
                    y[c] = 0.0f;
                    continue;
                }
                const int i = static_cast<int>(c % sy);
                const int j = static_cast<int>((c / sy) % static_cast<std::size_t>(d.y));
                const int k = static_cast<int>(c / sz);
                float s = diag[c] * x[c];
                if (i > 0 && !f.solid[c - sx])
                    s -= x[c - sx];
                if (i < d.x - 1 && !f.solid[c + sx])
                    s -= x[c + sx];
                if (j > 0 && !f.solid[c - sy])
                    s -= x[c - sy];
                if (j < d.y - 1 && !f.solid[c + sy])
                    s -= x[c + sy];
                if (k > 0 && !f.solid[c - sz])
                    s -= x[c - sz];
                if (k < d.z - 1 && !f.solid[c + sz])
                    s -= x[c + sz];
                y[c] = s;
            }
        });
    };
    auto dot = [&](const std::vector<float>& a, const std::vector<float>& b) {
        std::vector<double> partial(16, 0.0);
        std::atomic<int> slot{0};
        parallelFor(N, [&](std::size_t begin, std::size_t end) {
            double acc = 0.0;
            for (std::size_t c = begin; c < end; ++c)
                acc += static_cast<double>(a[c]) * b[c];
            partial[static_cast<std::size_t>(slot++)] = acc;
        });
        double sum = 0.0;
        for (double p : partial)
            sum += p;
        return sum;
    };

    // Initial guess: uniform freestream.
    std::vector<float> psi(N, 0.0f);
    for (int k = 0; k < d.z; ++k)
        for (int j = 0; j < d.y; ++j)
            for (int i = 0; i < d.x; ++i) {
                const std::size_t c = f.index(i, j, k);
                if (!f.solid[c])
                    psi[c] = f.origin.x + h * static_cast<float>(i);
            }

    std::vector<float> r(N), z(N), p(N), Ap(N);
    apply(psi, Ap);
    for (std::size_t c = 0; c < N; ++c) {
        r[c] = diag[c] > 0.0f ? rhs[c] - Ap[c] : 0.0f;
        z[c] = diag[c] > 0.0f ? r[c] / diag[c] : 0.0f;
        p[c] = z[c];
    }
    double rz = dot(r, z);
    const double tol2 = std::pow(1e-6 * static_cast<double>(h), 2) * static_cast<double>(N);
    int it = 0;
    const double r0 = std::max(dot(r, r), tol2);
    for (; it < 3000; ++it) {
        const double rr = dot(r, r);
        if (rr <= tol2)
            break;
        if (cancel && cancel->load())
            return {};
        if (progress && *progress && it % 25 == 0) {
            // Residual falls roughly geometrically: report progress on a log scale.
            const float frac = static_cast<float>(std::log(r0 / rr) / std::log(r0 / tol2));
            (*progress)({"Preview", std::clamp(frac, 0.0f, 0.99f), "Solving potential flow around the body"});
        }
        apply(p, Ap);
        const double alpha = rz / std::max(dot(p, Ap), 1e-30);
        parallelFor(N, [&](std::size_t begin, std::size_t end) {
            for (std::size_t c = begin; c < end; ++c) {
                psi[c] += static_cast<float>(alpha) * p[c];
                r[c] -= static_cast<float>(alpha) * Ap[c];
                z[c] = diag[c] > 0.0f ? r[c] / diag[c] : 0.0f;
            }
        });
        const double rzNew = dot(r, z);
        const float beta = static_cast<float>(rzNew / std::max(rz, 1e-30));
        rz = rzNew;
        parallelFor(N, [&](std::size_t begin, std::size_t end) {
            for (std::size_t c = begin; c < end; ++c)
                p[c] = z[c] + beta * p[c];
        });
    }
    if (std::getenv("WT_DEBUG_PREVIEW"))
        std::fprintf(stderr, "potential: %d CG iterations\n", it);
    return psi;
}

} // namespace

core::FlowField makeSyntheticField(const core::SurfaceMesh& body, const core::SimulationParams& params,
                                   const std::atomic<bool>* cancel, const core::ProgressFn* progress)
{
    const core::Bounds b = body.bounds();
    const core::TunnelDomain domain = core::makeTunnelDomain(b, params);
    core::FlowField f = core::makeFieldForDomain(domain);
    f.freestreamSpeed = params.inletSpeed;
    core::voxelizeSolid(body, f);

    const std::vector<float> psi = solvePotential(f, cancel, progress);
    if (psi.empty())
        return {};
    const glm::ivec3 d = f.dims;
    const float h = f.spacing.x;
    const float U = params.inletSpeed;

    // Velocity = U * grad(psi): central differences, one-sided next to walls
    // and solid cells (where the normal gradient is zero by construction).
    auto value = [&](int i, int j, int k, bool& ok) -> float {
        if (i < 0)
            return f.origin.x - h;
        if (i >= d.x)
            return f.origin.x + h * static_cast<float>(d.x);
        if (j < 0 || j >= d.y || k < 0 || k >= d.z || f.solid[f.index(i, j, k)]) {
            ok = false;
            return 0.0f;
        }
        return psi[f.index(i, j, k)];
    };
    auto derivative = [&](int i, int j, int k, const glm::ivec3& e) {
        bool okMinus = true, okPlus = true;
        const float c = psi[f.index(i, j, k)];
        const float m = value(i - e.x, j - e.y, k - e.z, okMinus);
        const float p = value(i + e.x, j + e.y, k + e.z, okPlus);
        if (okMinus && okPlus)
            return (p - m) / (2.0f * h);
        if (okPlus)
            return (p - c) / h;
        if (okMinus)
            return (c - m) / h;
        return 0.0f;
    };
    for (int k = 0; k < d.z; ++k)
        for (int j = 0; j < d.y; ++j)
            for (int i = 0; i < d.x; ++i) {
                const std::size_t n = f.index(i, j, k);
                if (f.solid[n])
                    continue;
                glm::vec3 u = U * glm::vec3(derivative(i, j, k, {1, 0, 0}), derivative(i, j, k, {0, 1, 0}),
                                            derivative(i, j, k, {0, 0, 1}));
                // Staircase corners of the voxelised body are singular in
                // potential flow; cap them so they don't dominate colour scales.
                const float speed = glm::length(u);
                if (speed > 2.0f * U)
                    u *= 2.0f * U / speed;
                f.velocity[n] = u;
                f.pressure[n] = 0.5f * (U * U - glm::dot(u, u));
            }
    return f;
}

void SyntheticSolver::setup(const core::SurfaceMesh& body, const core::SimulationParams& params)
{
    if (body.empty())
        throw std::runtime_error("Synthetic solver: body mesh is empty");
    body_ = body;
    params_ = params;
}

void SyntheticSolver::run(const core::ProgressFn& progress, const std::atomic<bool>& cancel)
{
    if (progress)
        progress({"Preview", 0.0f, "Voxelising the body"});
    field_ = makeSyntheticField(body_, params_, &cancel, &progress);
    if (progress && !field_.empty())
        progress({"Done", 1.0f, "Potential-flow preview ready"});
}

} // namespace solvers
