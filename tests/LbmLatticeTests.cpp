// D3Q19 velocity set, equilibrium and collision (host build of the code the
// GPU runs).

#include "Check.h"

#include "solvers/cuda/LbmLattice.h"

#include <random>

using namespace solvers::lbm;

namespace {

// Second moment sum_k c_ka c_kb g_k.
float secondMoment(const float (&g)[kQ], int a, int b)
{
    float s = 0.0f;
    for (int k = 0; k < kQ; ++k) {
        const int c[3] = {cx(k), cy(k), cz(k)};
        s += static_cast<float>(c[a] * c[b]) * g[k];
    }
    return s;
}

struct Dir {
    int x, y, z;
};

} // namespace

TEST(velocitySetIsSymmetricAndIsotropic)
{
    float wsum = 0.0f;
    for (int k = 0; k < kQ; ++k) {
        wsum += weight(k);
        const int o = opposite(k);
        CHECK(cx(o) == -cx(k) && cy(o) == -cy(k) && cz(o) == -cz(k));
        CHECK(opposite(o) == k);
        CHECK(direction(cx(k), cy(k), cz(k)) == k);
    }
    CHECK(direction(1, 1, 1) == -1); // not in D3Q19
    CHECK_NEAR(wsum, 1.0, 1e-6);
    float w[kQ];
    for (int k = 0; k < kQ; ++k)
        w[k] = weight(k);
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            CHECK_NEAR(secondMoment(w, a, b), a == b ? 1.0 / 3.0 : 0.0, 1e-6);
}

TEST(equilibriumHasTheRightMoments)
{
    const float rho = 1.02f, u[3] = {0.08f, -0.03f, 0.05f};
    float f[kQ];
    for (int k = 0; k < kQ; ++k)
        f[k] = equilibrium(k, rho, u[0], u[1], u[2]);
    const Moments m = moments(f);
    CHECK_NEAR(m.rho, rho, 1e-6);
    CHECK_NEAR(m.ux, u[0], 1e-6);
    CHECK_NEAR(m.uy, u[1], 1e-6);
    CHECK_NEAR(m.uz, u[2], 1e-6);
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            CHECK_NEAR(secondMoment(f, a, b), rho * ((a == b ? 1.0f / 3.0f : 0.0f) + u[a] * u[b]), 1e-6);
}

TEST(collisionConservesMassAndMomentum)
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> noise(-0.004f, 0.004f);
    for (int trial = 0; trial < 20; ++trial) {
        float f[kQ];
        for (int k = 0; k < kQ; ++k)
            f[k] = equilibrium(k, 1.0f, 0.07f, 0.01f, -0.02f) + noise(rng) * weight(k);
        const Moments before = moments(f);
        const float tau = collide(f, before, 0.5001f, 0.14f);
        const Moments after = moments(f);
        CHECK(tau >= 0.5001f);
        CHECK_NEAR(after.rho, before.rho, 2e-6);
        CHECK_NEAR(after.ux, before.ux, 2e-6);
        CHECK_NEAR(after.uy, before.uy, 2e-6);
        CHECK_NEAR(after.uz, before.uz, 2e-6);
    }
}

TEST(collisionLeavesEquilibriumAlone)
{
    float f[kQ], feq[kQ];
    for (int k = 0; k < kQ; ++k)
        f[k] = feq[k] = equilibrium(k, 0.99f, 0.05f, 0.0f, 0.02f);
    const float tau = collide(f, moments(f), 0.6f, 0.14f);
    CHECK_NEAR(tau, 0.6, 1e-5); // no strain, no eddy viscosity
    for (int k = 0; k < kQ; ++k)
        CHECK_NEAR(f[k], feq[k], 1e-6);
}

TEST(mrtBasisIsOrthogonal)
{
    for (int r = 0; r < kQ; ++r)
        for (int q = 0; q < kQ; ++q) {
            int dot = 0;
            for (int k = 0; k < kQ; ++k)
                dot += basis(r, k) * basis(q, k);
            CHECK(dot == (r == q ? basisNorm(r) : 0));
        }
    // Conserved moments are never relaxed; the shear moments follow 1 / tau.
    for (int r : {0, 3, 5, 7})
        CHECK(relaxationRate(r, 1.9f) == 0.0f);
    for (int r : {9, 11, 13, 14, 15})
        CHECK(relaxationRate(r, 1.9f) == 1.9f);
}

TEST(collisionRelaxesStressAndDampsGhostModes)
{
    // Equilibrium plus a non-equilibrium shear stress Pxy and a higher-order
    // ("ghost") part that carries no mass, momentum or stress at all.
    const float rho = 1.0f, ux = 0.06f;
    float f[kQ], shear[kQ], ghost[kQ] = {};
    const float pxy = 2e-4f;
    for (int k = 0; k < kQ; ++k) {
        shear[k] = 4.5f * weight(k) * 2.0f * static_cast<float>(cx(k) * cy(k)) * pxy;
        f[k] = equilibrium(k, rho, ux, 0.0f, 0.0f) + shear[k];
    }
    // Ghost: +g on the four xy diagonals, -2g on the four x/y axis links,
    // +4g at rest: only epsilon and pi moments, which both relax at 1.4.
    const float g = 1e-4f;
    for (const auto& d : {Dir{1, 1, 0}, Dir{-1, -1, 0}, Dir{1, -1, 0}, Dir{-1, 1, 0}})
        ghost[direction(d.x, d.y, d.z)] = g;
    for (const auto& d : {Dir{1, 0, 0}, Dir{-1, 0, 0}, Dir{0, 1, 0}, Dir{0, -1, 0}})
        ghost[direction(d.x, d.y, d.z)] = -2.0f * g;
    ghost[0] = 4.0f * g;
    for (int k = 0; k < kQ; ++k)
        f[k] += ghost[k];
    const Moments m = moments(f);
    CHECK_NEAR(m.rho, rho, 1e-6);
    CHECK_NEAR(secondMoment(ghost, 0, 0), 0.0, 1e-9);
    CHECK_NEAR(secondMoment(ghost, 0, 1), 0.0, 1e-9);

    const float tau0 = 0.8f;
    collide(f, m, tau0, 0.0f); // no eddy viscosity
    for (int k = 0; k < kQ; ++k) {
        const float neq = f[k] - equilibrium(k, rho, ux, 0.0f, 0.0f);
        // Stress relaxed by (1 - 1/tau); the ghost part by (1 - 1.4).
        CHECK_NEAR(neq, (1.0f - 1.0f / tau0) * shear[k] + (1.0f - kRatePi) * ghost[k], 2e-7);
    }
    CHECK(kRatePi == kRateEps); // the check above relies on this
}

TEST(wallShearFollowsTheLogLaw)
{
    // Viscous sublayer: u_tau^2 = nu u / y.
    CHECK_NEAR(wallShear(0.001f, 0.5f, 1e-3f), 1e-3 * 0.001 / 0.5, 1e-9);
    CHECK_NEAR(wallShear(0.0f, 0.5f, 1e-5f), 0.0, 0.0);
    CHECK_NEAR(wallShear(-1.0f, 0.5f, 1e-5f), 0.0, 0.0);
    // Log region: u / u_tau = ln(E y u_tau / nu) / kappa.
    const float u = 0.08f, y = 0.5f, nu = 2e-6f;
    const float utau = std::sqrt(wallShear(u, y, nu));
    CHECK(y * utau / nu > 11.0f);
    CHECK_NEAR(u / utau, std::log(8.43f * y * utau / nu) / 0.41f, 1e-3 * u / utau);
    CHECK(utau > 0.02f * u && utau < 0.06f * u); // a few per cent of the outer speed, as in real boundary layers
}

TEST(smagorinskyTauSolvesItsDefinition)
{
    const float tau0 = 0.50001f, cs = 0.14f, rho = 1.0f;
    CHECK_NEAR(smagorinskyTau(tau0, cs, 0.0f, rho), tau0, 1e-7);
    float previous = tau0;
    for (float pi : {1e-6f, 1e-5f, 1e-4f, 1e-3f}) {
        const float tau = smagorinskyTau(tau0, cs, pi, rho);
        CHECK(tau > previous);
        previous = tau;
        // tau = tau0 + 3 Cs^2 |S|, |S| = 3 sqrt(2) |Pi| / (2 rho tau)
        const float strain = 3.0f * std::sqrt(2.0f) * pi / (2.0f * rho * tau);
        CHECK_NEAR(tau, tau0 + 3.0f * cs * cs * strain, 1e-6);
    }
}
