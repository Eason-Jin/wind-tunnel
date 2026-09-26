// Physics of the lattice update on small CPU lattices: boundary conditions
// that must preserve a uniform stream, and an obstacle that must produce a
// stagnation point and a wake while conserving mass.

#include "Check.h"
#include "LbmReference.h"

#include "solvers/cuda/LbmSetup.h"

#include <algorithm>
#include <cmath>

using namespace solvers::lbm;

namespace {

LatticeParams smallLattice(int nx, int ny, int nz, bool movingFloor)
{
    LatticeParams p;
    p.nx = nx;
    p.ny = ny;
    p.nz = nz;
    p.uInlet = 0.08f;
    p.tau0 = 0.50002f; // effectively inviscid, as in a real run
    p.smagorinsky = kSmagorinsky;
    p.movingFloor = movingFloor;
    return p;
}

std::vector<std::uint8_t> boxObstacle(const LatticeParams& p, glm::ivec3 lo, glm::ivec3 hi)
{
    std::vector<std::uint8_t> solid(p.cells(), 0);
    for (int z = lo.z; z <= hi.z; ++z)
        for (int y = lo.y; y <= hi.y; ++y)
            for (int x = lo.x; x <= hi.x; ++x)
                solid[p.index(x, y, z)] = 1;
    return buildFlags(solid, {p.nx, p.ny, p.nz});
}

// Mass flux through the plane x = const (lattice units).
double massFlux(const test::CpuLattice& L, int x)
{
    double flux = 0.0;
    for (int z = 0; z < L.p.nz; ++z)
        for (int y = 0; y < L.p.ny; ++y) {
            if (L.flags[L.p.index(x, y, z)] & kSolid)
                continue;
            const Moments m = L.at(x, y, z);
            flux += static_cast<double>(m.rho) * m.ux;
        }
    return flux;
}

} // namespace

TEST(uniformStreamIsPreservedByEveryBoundary)
{
    // With no body, inlet, outlet, slip walls and a floor moving at the free
    // stream speed are all consistent with uniform flow: it must not change.
    for (bool moving : {true, false}) {
        const LatticeParams p = smallLattice(20, 9, 7, moving);
        test::CpuLattice L(p, buildFlags(std::vector<std::uint8_t>(p.cells(), 0), {p.nx, p.ny, p.nz}));
        L.advance(150);
        float worstU = 0.0f, worstRho = 0.0f;
        for (int z = 0; z < p.nz; ++z)
            for (int y = 0; y < p.ny; ++y)
                for (int x = 0; x < p.nx; ++x) {
                    const Moments m = L.at(x, y, z);
                    worstU = std::max({worstU, std::abs(m.ux - p.uInlet), std::abs(m.uy), std::abs(m.uz)});
                    worstRho = std::max(worstRho, std::abs(m.rho - 1.0f));
                }
        CHECK(worstU < 1e-5f);
        CHECK(worstRho < 1e-5f);
    }
}

TEST(obstacleMakesStagnationPointAndWake)
{
    const LatticeParams p = smallLattice(56, 20, 16, false);
    // A cube in mid stream, 5 cells a side, a quarter of the way down the tunnel.
    const glm::ivec3 lo{14, 8, 6}, hi{18, 12, 10};
    test::CpuLattice L(p, boxObstacle(p, lo, hi));
    L.advance(900);
    L.advance(500, 2);
    CHECK(L.samples == 250);

    // Nothing blew up.
    bool finite = true;
    for (float v : L.fA)
        finite = finite && std::isfinite(v);
    CHECK(finite);

    const int yc = (lo.y + hi.y) / 2, zc = (lo.z + hi.z) / 2;
    auto mean = [&](int x, int y, int z, int q) {
        return L.sums[static_cast<std::size_t>(q) * p.cells() + p.index(x, y, z)] / static_cast<float>(L.samples);
    };
    // Stagnation: the cell in front of the face is nearly at rest and at a
    // large part of the dynamic pressure (Cp = (rho - 1) cs^2 / (u^2 / 2);
    // the cube is only 5 cells wide, so the cell centre a cell ahead of it
    // does not see the full Cp = 1), while the air beside it is in suction.
    const float q = 0.5f * p.uInlet * p.uInlet;
    const float cpFront = mean(lo.x - 1, yc, zc, 3) / 3.0f / q;
    const float cpSide = mean((lo.x + hi.x) / 2, hi.y + 1, zc, 3) / 3.0f / q;
    CHECK(mean(lo.x - 1, yc, zc, 0) < 0.2f * p.uInlet);
    CHECK(cpFront > 0.4f && cpFront < 1.3f);
    CHECK(cpSide < 0.0f);
    // Wake: slow (or reversed) flow just behind the cube, recovering downstream.
    CHECK(mean(hi.x + 2, yc, zc, 0) < 0.3f * p.uInlet);
    CHECK(mean(hi.x + 25, yc, zc, 0) > mean(hi.x + 2, yc, zc, 0));
    // Flow speeds up around the sides.
    CHECK(mean((lo.x + hi.x) / 2, hi.y + 2, zc, 0) > 1.05f * p.uInlet);

    // Mass conservation: the flux through a plane ahead of the body matches
    // one behind it (fixed inlet velocity, so the outlet must carry the same).
    const double in = massFlux(L, 4), out = massFlux(L, 48);
    CHECK_NEAR(out / in, 1.0, 0.02);
}

TEST(boundaryRulesPickTheRightPopulation)
{
    // A 4x4x4 lattice with a solid cell at (2,1,1); every stored population
    // gets a unique value so we can see exactly which one each rule reads.
    LatticeParams p = smallLattice(4, 4, 4, true);
    std::vector<std::uint8_t> solid(p.cells(), 0);
    solid[p.index(2, 1, 1)] = 1;
    const auto flags = buildFlags(solid, {p.nx, p.ny, p.nz});
    const std::size_t n = p.cells();
    std::vector<float> fIn(kQ * n);
    for (std::size_t i = 0; i < fIn.size(); ++i)
        fIn[i] = 0.001f * static_cast<float>(i);
    auto stored = [&](int k, int x, int y, int z) { return fIn[static_cast<std::size_t>(k) * n + p.index(x, y, z)]; };
    Moments own;
    own.rho = 1.01f;
    own.ux = 0.05f;
    own.uy = 0.01f;
    own.uz = -0.02f;

    // Interior link: plain streaming from the upstream neighbour.
    const int kx = direction(1, 0, 0);
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 1, 2, 2, kx, own), stored(kx, 0, 2, 2), 0.0);
    // Body: halfway bounce-back of this cell's own opposite population.
    const int kFromSolid = direction(-1, 0, 0); // arriving at (1,1,1) from (2,1,1)
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 1, 1, 1, kFromSolid, own), stored(opposite(kFromSolid), 1, 1, 1), 0.0);
    // ... sliding with the wall-model velocity: bounce-back plus 6 w rho (c . u_wall).
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 1, 1, 1, kFromSolid, own, 0.02f, 0.03f, 0.0f),
               stored(opposite(kFromSolid), 1, 1, 1) + 6.0f * weight(kFromSolid) * own.rho * (-1.0f * 0.02f), 1e-7);
    // Inlet: equilibrium at the inlet velocity and the cell's own density.
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 0, 2, 2, kx, own), equilibrium(kx, own.rho, p.uInlet, 0, 0), 1e-7);
    // Outlet: equilibrium at the reference density and the cell's own velocity.
    const int kBack = direction(-1, 1, 0);
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 3, 2, 2, kBack, own), equilibrium(kBack, 1.0f, own.ux, own.uy, own.uz), 1e-7);
    // ... never with back-flow.
    Moments backwards = own;
    backwards.ux = -0.03f;
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 3, 2, 2, kBack, backwards), equilibrium(kBack, 1.0f, 0.0f, own.uy, own.uz), 1e-7);
    // Moving floor: bounce-back plus the wall's momentum, 6 w (c . u_wall).
    const int kUp = direction(1, 0, 1);
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 2, 2, 0, kUp, own),
               stored(opposite(kUp), 2, 2, 0) + 6.0f * weight(kUp) * p.uInlet, 1e-7);
    // Slip floor: specular reflection from the in-plane upstream neighbour.
    p.movingFloor = false;
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 2, 2, 0, kUp, own), stored(direction(1, 0, -1), 1, 2, 0), 0.0);
    // Slip side wall (y = 0), diagonal link.
    const int kSide = direction(1, 1, 0);
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 2, 0, 2, kSide, own), stored(direction(1, -1, 0), 1, 0, 2), 0.0);
    // Slip top wall, straight down.
    const int kDown = direction(0, 0, -1);
    CHECK_NEAR(incoming(p, fIn.data(), flags.data(), 2, 2, 3, kDown, own), stored(direction(0, 0, 1), 2, 2, 3), 0.0);
    // Specular source inside the body falls back to bounce-back: cell (3,1,0)
    // reflecting off the slip floor would read (2,1,0)... which is fluid; put
    // a solid there to check the fallback.
    solid[p.index(2, 1, 0)] = 1;
    const auto flags2 = buildFlags(solid, {p.nx, p.ny, p.nz});
    CHECK_NEAR(incoming(p, fIn.data(), flags2.data(), 3, 1, 0, kUp, own), stored(opposite(kUp), 3, 1, 0), 0.0);
}

TEST(wallModelOnlyWhereTheBodyIsOnOneSide)
{
    // Solid cells in a 7x7x7 box; wallGeometry is asked about cell (3,3,3).
    const LatticeParams p = smallLattice(7, 7, 7, true);
    struct Wall {
        bool modelled = false;
        glm::vec3 normal{0.0f};
        float area = 0.0f;
    };
    auto wallAt = [&](const std::vector<glm::ivec3>& solids) {
        std::vector<std::uint8_t> flags(p.cells(), kFluid);
        for (const glm::ivec3& s : solids)
            flags[p.index(s.x, s.y, s.z)] = kSolid;
        Wall w;
        w.modelled = wallGeometry(p, flags.data(), 3, 3, 3, w.normal.x, w.normal.y, w.normal.z, w.area);
        return w;
    };
    auto slab = [](int z, std::vector<glm::ivec3> v = {}) {
        for (int y = 2; y <= 4; ++y)
            for (int x = 2; x <= 4; ++x)
                v.push_back({x, y, z});
        return v;
    };

    // No body in reach.
    CHECK(!wallAt({}).modelled);
    // Flat floor below: normal straight up, one face of wall.
    const Wall floor = wallAt(slab(2));
    CHECK(floor.modelled);
    CHECK_NEAR(floor.normal.z, 1.0, 1e-6);
    CHECK_NEAR(floor.area, 1.0, 1e-5);
    // A lone solid cell on a diagonal link still gives a direction.
    const Wall edge = wallAt({{3, 2, 2}});
    CHECK(edge.modelled);
    CHECK_NEAR(edge.normal.y, std::sqrt(0.5), 1e-6);
    CHECK_NEAR(edge.normal.z, std::sqrt(0.5), 1e-6);
    // Inside corner (floor and a wall at x = 2): normal along the bisector.
    std::vector<glm::ivec3> corner = slab(2);
    for (int z = 3; z <= 4; ++z)
        for (int y = 2; y <= 4; ++y)
            corner.push_back({2, y, z});
    const Wall inside = wallAt(corner);
    CHECK(inside.modelled);
    CHECK_NEAR(inside.normal.x, std::sqrt(0.5), 1e-6);
    CHECK_NEAR(inside.normal.z, std::sqrt(0.5), 1e-6);
    // One-cell slot between a floor and a ceiling: the links cancel.
    CHECK(!wallAt(slab(4, slab(2))).modelled);
    // Floor plus a fragment above and to one side: a link from the far side.
    std::vector<glm::ivec3> pocket = slab(2);
    pocket.push_back({4, 3, 4});
    CHECK(!wallAt(pocket).modelled);
}
