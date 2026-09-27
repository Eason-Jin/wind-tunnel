#pragma once

// D3Q19 lattice-Boltzmann core shared by the CUDA kernels (LbmDevice.cu) and
// host code (tests, CPU reference). Everything here is header-only and
// compiles as plain C++ as well as CUDA, so the exact per-cell update the GPU
// runs can be exercised on the CPU.
//
// Scheme (lattice units throughout: dx = dt = 1, reference density 1):
// - two-lattice pull streaming;
// - multiple-relaxation-time collision (d'Humieres et al. 2002). At the
//   Reynolds numbers of a car in a wind tunnel the molecular relaxation time
//   is within 1e-5 of 1/2, where BGK and projective-regularised collisions
//   were found to go unstable at walls or even in free stream; MRT's damped
//   ghost and bulk (sound) modes keep the run stable;
// - a Smagorinsky eddy viscosity from the local non-equilibrium stress (no
//   finite differences);
// - bounce-back walls on the voxelised body with a log-law wall model: the
//   wall moves tangentially with the adjacent air (so the unresolved
//   boundary layer does not act as a laminar no-slip wall) and the wall
//   shear stress from the log law is applied as a force instead. Without it
//   the flow separates as if laminar, giving a far longer wake than a
//   turbulent boundary layer at these Reynolds numbers.

#include <cmath>
#include <cstddef>
#include <cstdint>

#if defined(__CUDACC__)
#define WT_HD __host__ __device__ __forceinline__
#else
#define WT_HD inline
#endif
// Fully unrolled loops over directions/moments let the compiler fold the
// constant velocity set and MRT matrix away.
#if defined(__CUDA_ARCH__) || defined(__clang__)
#define WT_UNROLL _Pragma("unroll")
#elif defined(__GNUC__) && !defined(__CUDACC__) // (nvcc's host pass rejects the GCC spelling)
#define WT_UNROLL _Pragma("GCC unroll 19")
#else
#define WT_UNROLL
#endif

namespace solvers::lbm {

constexpr int kQ = 19;

// Velocity set: rest, then opposite pairs (k, k + 1) for odd k.
WT_HD constexpr int cx(int k)
{
    constexpr int v[kQ] = {0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0};
    return v[k];
}
WT_HD constexpr int cy(int k)
{
    constexpr int v[kQ] = {0, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 1, -1};
    return v[k];
}
WT_HD constexpr int cz(int k)
{
    constexpr int v[kQ] = {0, 0, 0, 0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, -1, 1, -1, 1};
    return v[k];
}
WT_HD constexpr float weight(int k) { return k == 0 ? 1.0f / 3.0f : k <= 6 ? 1.0f / 18.0f : 1.0f / 36.0f; }
WT_HD constexpr int opposite(int k) { return k == 0 ? 0 : (k & 1) ? k + 1 : k - 1; }

// Index of the direction with velocity (x, y, z), or -1 if there is none.
WT_HD constexpr int direction(int x, int y, int z)
{
    for (int k = 0; k < kQ; ++k)
        if (cx(k) == x && cy(k) == y && cz(k) == z)
            return k;
    return -1;
}

// Per-cell flags.
enum : std::uint8_t {
    kFluid = 0,
    kSolid = 1,        // inside the body: not updated, fluid neighbours bounce off it
    kNearBoundary = 2, // fluid cell with a solid or out-of-domain neighbour (takes the careful path)
};

// Tunnel box boundaries: velocity inlet at x = 0, pressure outlet at
// x = nx - 1, slip walls on the sides and top, and a floor that is either a
// no-slip wall moving with the air (a rolling-road ground plane) or slip.
// Walls sit half a cell outside the outermost cell centres.
struct LatticeParams {
    int nx = 0, ny = 0, nz = 0;
    float uInlet = 0.08f;      // inlet speed along +x
    float tau0 = 0.5f;         // relaxation time of the molecular viscosity (3 nu + 1/2)
    float smagorinsky = 0.14f; // Smagorinsky constant Cs (0 disables the eddy viscosity)
    bool movingFloor = true;   // floor is a no-slip wall moving at uInlet; otherwise slip
    bool wallModel = true;     // log-law wall model on the body; otherwise plain no-slip bounce-back

    WT_HD std::size_t cells() const { return static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz); }
    WT_HD std::size_t index(int x, int y, int z) const
    {
        return static_cast<std::size_t>(x) + static_cast<std::size_t>(nx) * (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(z));
    }
};

// Second-order (low Mach) equilibrium.
WT_HD float equilibrium(int k, float rho, float ux, float uy, float uz)
{
    const float cu = static_cast<float>(cx(k)) * ux + static_cast<float>(cy(k)) * uy + static_cast<float>(cz(k)) * uz;
    const float uu = ux * ux + uy * uy + uz * uz;
    return weight(k) * rho * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * uu);
}

struct Moments {
    float rho = 1.0f;
    float ux = 0.0f, uy = 0.0f, uz = 0.0f;
};

WT_HD Moments moments(const float (&f)[kQ])
{
    Moments m{0.0f, 0.0f, 0.0f, 0.0f};
    WT_UNROLL
    for (int k = 0; k < kQ; ++k) {
        m.rho += f[k];
        m.ux += static_cast<float>(cx(k)) * f[k];
        m.uy += static_cast<float>(cy(k)) * f[k];
        m.uz += static_cast<float>(cz(k)) * f[k];
    }
    const float inv = 1.0f / m.rho;
    m.ux *= inv;
    m.uy *= inv;
    m.uz *= inv;
    return m;
}

// Effective relaxation time with a Smagorinsky eddy viscosity, given the
// Frobenius norm |Pi_neq| of the non-equilibrium momentum flux. Solves
// tau = tau0 + 3 Cs^2 |S| with |S| = 3 sqrt(2) |Pi_neq| / (2 rho tau), i.e.
// the strain rate implied by the stress at the relaxation time being found.
WT_HD float smagorinskyTau(float tau0, float cs, float piNorm, float rho)
{
    return 0.5f * (tau0 + std::sqrt(tau0 * tau0 + 18.0f * 1.41421356f * cs * cs * piNorm / rho));
}

// MRT moment basis for D3Q19 (d'Humieres et al. 2002): row r of the
// transformation matrix evaluated for direction k. Rows are mutually
// orthogonal; basisNorm(r) is the squared norm of row r.
//   0 rho, 1 e, 2 eps, 3/5/7 j, 4/6/8 q, 9 3pxx, 10 3pixx, 11 pww, 12 piww,
//   13 pxy, 14 pyz, 15 pxz, 16..18 m
WT_HD constexpr int basis(int r, int k)
{
    const int x = cx(k), y = cy(k), z = cz(k);
    const int c2 = x * x + y * y + z * z;
    switch (r) {
    case 0: return 1;
    case 1: return 19 * c2 - 30;
    case 2: return (21 * c2 * c2 - 53 * c2 + 24) / 2;
    case 3: return x;
    case 4: return (5 * c2 - 9) * x;
    case 5: return y;
    case 6: return (5 * c2 - 9) * y;
    case 7: return z;
    case 8: return (5 * c2 - 9) * z;
    case 9: return 3 * x * x - c2;
    case 10: return (3 * c2 - 5) * (3 * x * x - c2);
    case 11: return y * y - z * z;
    case 12: return (3 * c2 - 5) * (y * y - z * z);
    case 13: return x * y;
    case 14: return y * z;
    case 15: return x * z;
    case 16: return (y * y - z * z) * x;
    case 17: return (z * z - x * x) * y;
    default: return (x * x - y * y) * z;
    }
}
WT_HD constexpr int basisNorm(int r)
{
    constexpr int v[kQ] = {19, 2394, 252, 10, 40, 10, 40, 10, 40, 36, 72, 12, 24, 4, 4, 4, 8, 8, 8};
    return v[r];
}

// Relaxation rates of the non-hydrodynamic moments. The energy moment sets
// the bulk viscosity: relaxing it quickly damps sound waves, which would
// otherwise ring undamped between the inlet, outlet and walls at the tiny
// shear viscosity of a high-Reynolds-number run. The ghost moments get the
// usual stable values (Lallemand & Luo 2000, d'Humieres et al. 2002).
constexpr float kRateBulk = 1.19f; // e
constexpr float kRateEps = 1.4f;   // epsilon
constexpr float kRateHeat = 1.2f;  // q
constexpr float kRatePi = 1.4f;    // pi_xx, pi_ww
constexpr float kRateM = 1.98f;    // m

// Relaxation rate of moment r when the shear rate is sNu (= 1 / tau).
WT_HD constexpr float relaxationRate(int r, float sNu)
{
    switch (r) {
    case 1: return kRateBulk;
    case 2: return kRateEps;
    case 4: case 6: case 8: return kRateHeat;
    case 9: case 11: case 13: case 14: case 15: return sNu;
    case 10: case 12: return kRatePi;
    case 16: case 17: case 18: return kRateM;
    default: return 0.0f; // conserved: rho, j
    }
}

// Multiple-relaxation-time collision in place, with the shear relaxation
// time raised by a Smagorinsky eddy viscosity. Mass and momentum are
// conserved exactly (up to rounding); returns the shear relaxation time used.
WT_HD float collide(float (&f)[kQ], const Moments& m, float tau0, float cs)
{
    float feq[kQ], neq[kQ];
    float pxx = 0.0f, pyy = 0.0f, pzz = 0.0f, pxy = 0.0f, pxz = 0.0f, pyz = 0.0f;
    WT_UNROLL
    for (int k = 0; k < kQ; ++k) {
        feq[k] = equilibrium(k, m.rho, m.ux, m.uy, m.uz);
        neq[k] = f[k] - feq[k];
        const float x = static_cast<float>(cx(k)), y = static_cast<float>(cy(k)), z = static_cast<float>(cz(k));
        pxx += x * x * neq[k];
        pyy += y * y * neq[k];
        pzz += z * z * neq[k];
        pxy += x * y * neq[k];
        pxz += x * z * neq[k];
        pyz += y * z * neq[k];
    }
    // The eddy viscosity follows the deviatoric (shear) part of the stress.
    const float trace = (pxx + pyy + pzz) * (1.0f / 3.0f);
    const float dxx = pxx - trace, dyy = pyy - trace, dzz = pzz - trace;
    const float piNorm = std::sqrt(dxx * dxx + dyy * dyy + dzz * dzz + 2.0f * (pxy * pxy + pxz * pxz + pyz * pyz));
    const float tau = cs > 0.0f ? smagorinskyTau(tau0, cs, piNorm, m.rho) : tau0;
    const float sNu = 1.0f / tau;

    // Non-equilibrium moments, relaxed, then back to populations:
    // f = feq + M^-1 (1 - S) M (f - feq), with M^-1 = M^T diag(1 / norm).
    float mneq[kQ];
    WT_UNROLL
    for (int r = 0; r < kQ; ++r) {
        float sum = 0.0f;
        WT_UNROLL
        for (int k = 0; k < kQ; ++k)
            sum += static_cast<float>(basis(r, k)) * neq[k];
        mneq[r] = sum * (1.0f - relaxationRate(r, sNu)) / static_cast<float>(basisNorm(r));
    }
    WT_UNROLL
    for (int k = 0; k < kQ; ++k) {
        float sum = 0.0f;
        WT_UNROLL
        for (int r = 0; r < kQ; ++r)
            sum += static_cast<float>(basis(r, k)) * mneq[r];
        f[k] = feq[k] + sum;
    }
    return tau;
}

// Kinematic wall shear stress u_tau^2 for tangential speed ut at distance y
// from a wall (lattice units): the log law u / u_tau = ln(E y+) / kappa, or
// the viscous sublayer u+ = y+ when that puts y+ below 11.
WT_HD float wallShear(float ut, float y, float nu)
{
    if (!(ut > 0.0f))
        return 0.0f;
    if (y * y * ut / nu < 121.0f) // y+ < 11 with the laminar friction velocity
        return nu * ut / y;
    constexpr float kappa = 0.41f, E = 8.43f; // E = exp(kappa B), B = 5.2
    float utau = 0.05f * ut;
    for (int i = 0; i < 6; ++i) // fixed point; contracts fast in the log region
        utau = kappa * ut / std::log(E * y * utau / nu);
    return utau * utau;
}

// The wall model applies only where every solid link c_k has c_k . n above
// this, n being the estimated wall normal: 1 on a flat wall, 0 for the
// diagonal links in a right-angled inside corner, negative for a link from
// the far side of a slot or pocket.
constexpr float kWallModelOneSided = -0.2f;

// Wall normal (into the air, unit length) and wall area (in cell faces) of
// fluid cell (x, y, z) next to the body. The links from the body sum, weighted,
// to a vector pointing away from the wall (1/6 of the unit normal on a flat
// wall). Returns false where the cell has no single wall to model: no solid
// links, or some of them from the far side of a slot, pocket or thin fragment,
// where a sliding wall would feed momentum in rather than take it out. Such
// cells keep plain no-slip bounce-back.
WT_HD bool wallGeometry(const LatticeParams& p, const std::uint8_t* flags, int x, int y, int z, float& nx, float& ny,
                        float& nz, float& area)
{
    std::uint32_t solidLinks = 0;
    nx = ny = nz = area = 0.0f;
    WT_UNROLL
    for (int k = 1; k < kQ; ++k) {
        const int sx = x - cx(k), sy = y - cy(k), sz = z - cz(k);
        if (sx >= 0 && sx < p.nx && sy >= 0 && sy < p.ny && sz >= 0 && sz < p.nz && (flags[p.index(sx, sy, sz)] & kSolid)) {
            solidLinks |= 1u << k;
            nx += weight(k) * static_cast<float>(cx(k));
            ny += weight(k) * static_cast<float>(cy(k));
            nz += weight(k) * static_cast<float>(cz(k));
        }
    }
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!(len > 1e-6f))
        return false;
    nx /= len;
    ny /= len;
    nz /= len;
    area = 6.0f * len < 3.0f ? 6.0f * len : 3.0f;
    bool oneSided = true;
    WT_UNROLL
    for (int k = 1; k < kQ; ++k)
        if ((solidLinks >> k) & 1u)
            oneSided = oneSided && static_cast<float>(cx(k)) * nx + static_cast<float>(cy(k)) * ny +
                                           static_cast<float>(cz(k)) * nz > kWallModelOneSided;
    return oneSided;
}

// Population arriving at fluid cell (x, y, z) with velocity c_k, for a cell
// next to a solid or the tunnel walls. fIn holds last step's post-collision
// populations (structure of arrays: fIn[k * cells + cell]); `own` is this
// cell's last state and (wx, wy, wz) the velocity the body wall moves with.
WT_HD float incoming(const LatticeParams& p, const float* fIn, const std::uint8_t* flags, int x, int y, int z, int k,
                     const Moments& own, float wx = 0.0f, float wy = 0.0f, float wz = 0.0f)
{
    const std::size_t n = p.cells();
    const std::size_t c = p.index(x, y, z);
    const int sx = x - cx(k), sy = y - cy(k), sz = z - cz(k);
    if (sx < 0) // inlet: fixed velocity, density taken from the cell itself
        return equilibrium(k, own.rho, p.uInlet, 0.0f, 0.0f);
    if (sx >= p.nx) // outlet: fixed (reference) density, velocity taken from the cell
        return equilibrium(k, 1.0f, own.ux > 0.0f ? own.ux : 0.0f, own.uy, own.uz);
    const bool outY = sy < 0 || sy >= p.ny;
    const bool outFloor = sz < 0;
    const bool outTop = sz >= p.nz;
    if (outFloor && p.movingFloor) // no-slip floor moving at the inlet speed (Ladd's moving bounce-back)
        return fIn[static_cast<std::size_t>(opposite(k)) * n + c] + 6.0f * weight(k) * static_cast<float>(cx(k)) * p.uInlet;
    if (outY || outFloor || outTop) {
        // Slip wall: specular reflection. The population left the in-plane
        // neighbour m with the wall-normal component(s) reversed.
        const bool outZ = outFloor || outTop;
        // Each candidate is a constant once the caller's loop over k is unrolled.
        const int rk = outY ? (outZ ? direction(cx(k), -cy(k), -cz(k)) : direction(cx(k), -cy(k), cz(k)))
                            : direction(cx(k), cy(k), -cz(k));
        const int my = outY ? y : sy, mz = outZ ? z : sz;
        const std::size_t m = p.index(sx, my, mz);
        if (flags[m] & kSolid)
            return fIn[static_cast<std::size_t>(opposite(k)) * n + c];
        return fIn[static_cast<std::size_t>(rk) * n + m];
    }
    const std::size_t s = p.index(sx, sy, sz);
    if (flags[s] & kSolid) // halfway bounce-back off the body, moving at (wx, wy, wz)
        return fIn[static_cast<std::size_t>(opposite(k)) * n + c] +
               6.0f * weight(k) * own.rho *
                   (static_cast<float>(cx(k)) * wx + static_cast<float>(cy(k)) * wy + static_cast<float>(cz(k)) * wz);
    return fIn[static_cast<std::size_t>(k) * n + s];
}

// One stream + collide update of cell (x, y, z): pulls from fIn, writes the
// post-collision populations to fOut. When `sums` is given (structure of
// arrays: ux, uy, uz, rho - 1, each `cells` long) the cell's velocity and
// density are added to it for time averaging. Solid cells are left alone.
WT_HD void updateCell(const LatticeParams& p, const float* fIn, float* fOut, const std::uint8_t* flags, float* sums, int x,
                      int y, int z)
{
    const std::size_t c = p.index(x, y, z);
    const std::uint8_t flag = flags[c];
    if (flag & kSolid)
        return;
    const std::size_t n = p.cells();
    float f[kQ];
    bool wall = false;                     // wall-modelled cell next to the body
    float nx = 0.0f, ny = 0.0f, nz = 0.0f; // its wall normal (into the air)
    float area = 0.0f;                     // its wall area (in cell faces)
    if (!(flag & kNearBoundary)) {
        const long long sx = 1, sy = p.nx, sz = static_cast<long long>(p.nx) * p.ny;
        WT_UNROLL
        for (int k = 0; k < kQ; ++k) {
            const long long offset = cx(k) * sx + cy(k) * sy + cz(k) * sz;
            f[k] = fIn[static_cast<std::size_t>(k) * n + static_cast<std::size_t>(static_cast<long long>(c) - offset)];
        }
    } else {
        if (p.wallModel)
            wall = wallGeometry(p, flags, x, y, z, nx, ny, nz, area);
        // The inlet, outlet and wall rules need this cell's own state, which
        // last step's post-collision populations still carry (collision
        // conserves mass and momentum).
        Moments own;
        if (x == 0 || x == p.nx - 1 || wall) {
            float g[kQ];
            WT_UNROLL
            for (int k = 0; k < kQ; ++k)
                g[k] = fIn[static_cast<std::size_t>(k) * n + c];
            own = moments(g);
        }
        // The body wall slides with the air's tangential velocity.
        float wx = 0.0f, wy = 0.0f, wz = 0.0f;
        if (wall) {
            const float un = own.ux * nx + own.uy * ny + own.uz * nz;
            wx = own.ux - un * nx;
            wy = own.uy - un * ny;
            wz = own.uz - un * nz;
        }
        WT_UNROLL
        for (int k = 0; k < kQ; ++k)
            f[k] = incoming(p, fIn, flags, x, y, z, k, own, wx, wy, wz);
    }
    const Moments m = moments(f);
    collide(f, m, p.tau0, p.smagorinsky);
    if (wall) {
        // Wall friction from the log law (first cell centre half a cell
        // from the wall), applied as a velocity change against the
        // tangential flow; never more than half of it in one step.
        const float un = m.ux * nx + m.uy * ny + m.uz * nz;
        const float tx = m.ux - un * nx, ty = m.uy - un * ny, tz = m.uz - un * nz;
        const float ut = std::sqrt(tx * tx + ty * ty + tz * tz);
        float du = area * wallShear(ut, 0.5f, (p.tau0 - 0.5f) / 3.0f);
        du = du < 0.5f * ut ? du : 0.5f * ut;
        const float s = ut > 0.0f ? du / ut : 0.0f;
        WT_UNROLL
        for (int k = 0; k < kQ; ++k)
            f[k] += equilibrium(k, m.rho, m.ux - s * tx, m.uy - s * ty, m.uz - s * tz) - equilibrium(k, m.rho, m.ux, m.uy, m.uz);
    }
    WT_UNROLL
    for (int k = 0; k < kQ; ++k)
        fOut[static_cast<std::size_t>(k) * n + c] = f[k];
    if (sums) {
        sums[c] += m.ux;
        sums[n + c] += m.uy;
        sums[2 * n + c] += m.uz;
        sums[3 * n + c] += m.rho - 1.0f;
    }
}

// Initial state: every fluid cell at equilibrium with the inlet velocity,
// solid cells at rest.
WT_HD void initialiseCell(const LatticeParams& p, float* f, const std::uint8_t* flags, std::size_t c)
{
    const std::size_t n = p.cells();
    const float u = (flags[c] & kSolid) ? 0.0f : p.uInlet;
    WT_UNROLL
    for (int k = 0; k < kQ; ++k)
        f[static_cast<std::size_t>(k) * n + c] = equilibrium(k, 1.0f, u, 0.0f, 0.0f);
}

} // namespace solvers::lbm
