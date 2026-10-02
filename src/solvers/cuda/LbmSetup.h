#pragma once

// Host-side set-up for the lattice-Boltzmann solver: unit conversion, lattice
// sizing, boundary flags and the mapping of the time-averaged lattice fields
// back onto the app's output grid. No CUDA here, so all of it builds (and is
// tested) without a GPU.

#include "core/FlowField.h"
#include "core/SimulationParams.h"
#include "core/SurfaceMesh.h"
#include "core/TunnelDomain.h"
#include "solvers/cuda/LbmLattice.h"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace solvers::lbm {

// Inlet speed in lattice units. Mach 0.14 (cs = 1/sqrt(3)): low enough that
// compressibility errors stay well under 1% of the dynamic pressure, high
// enough to keep the number of time steps down.
constexpr float kLatticeInletSpeed = 0.08f;
constexpr float kSmagorinsky = 0.14f;
// Air gaps up to this many lattice cells high between the body and a moving
// ground are filled (see closeFloorGaps()).
constexpr int kFloorGapCells = 1;

// Conversion between lattice and physical units for one run.
struct LatticeUnits {
    float dx = 1.0f;        // lattice spacing (m)
    float dt = 1.0f;        // time step (s)
    float uLattice = kLatticeInletSpeed;
    float nuLattice = 0.0f; // molecular kinematic viscosity (lattice units)
    float tau0 = 0.5f;      // BGK relaxation time for nuLattice

    float speedScale() const { return dx / dt; } // m/s per lattice speed unit
    float velocity(float uLat) const { return uLat * speedScale(); }
    // Kinematic gauge pressure p/rho (m^2/s^2) of a lattice density deviation
    // rho - 1 (the outlet's reference density): p = cs^2 (rho - 1) with cs^2 = 1/3.
    float kinematicPressure(float rhoMinusOne) const { return rhoMinusOne / 3.0f * speedScale() * speedScale(); }
};

// Units for lattice spacing `dx` (m), physical inlet speed (m/s) and
// kinematic viscosity (m^2/s), with the inlet at `uLattice`.
LatticeUnits makeLatticeUnits(float dx, float inletSpeed, float kinematicViscosity, float uLattice = kLatticeInletSpeed);

// Everything the solver decides before touching the GPU.
struct LbmPlan {
    core::TunnelDomain domain;   // output grid (identical to the other solvers')
    int refine = 2;              // lattice cells per output cell along each axis
    glm::ivec3 lattice{0};       // lattice cells
    glm::vec3 latticeOrigin{0};  // centre of lattice cell (0,0,0)
    LatticeUnits units;
    bool movingFloor = true;     // ground plane: no-slip floor moving with the air
    int steps = 0;               // time steps in the run
    int averageFrom = 0;         // first step whose state goes into the time average
    int averageEvery = 4;        // sample the average every this many steps
    int clipFrames = 0;          // instantaneous snapshots to record (0 = no clip)
    int clipEvery = 1;           // steps between snapshots
    int clipFrom = 0;            // step of the first snapshot; the last falls on `steps`
    int clipSmoothing = 1;       // steps averaged into each snapshot, ending on its step
    std::size_t deviceBytes = 0; // GPU memory the lattice needs

    std::size_t cells() const { return static_cast<std::size_t>(lattice.x) * lattice.y * lattice.z; }
    LatticeParams latticeParams() const;
};

// Bytes of GPU memory for a lattice of `cells` cells: two population sets,
// flags and four time-average sums.
std::size_t latticeDeviceBytes(std::size_t cells);

// Host memory a recorded clip may use, and its frame limits.
constexpr std::size_t kClipBudgetBytes = 512u << 20;
constexpr int kClipMaxFrames = 96;
constexpr int kClipMinFrames = 12;

LbmPlan makeLbmPlan(const core::Bounds& body, const core::SimulationParams& params);

// Fill thin air gaps between the body and the floor: in every column where
// the body comes within `cells` lattice cells of the floor, the cells below
// it become solid. A body resting on a moving ground otherwise meets it in a
// wedge only a cell or two high, where the ground drags air into a gap the
// lattice cannot resolve and the pressure spikes (the lattice counterpart of
// the contact patch the OpenFOAM case cuts into the floor).
void closeFloorGaps(std::vector<std::uint8_t>& solid, const glm::ivec3& dims, int cells);

// Lattice flags from a solid mask (x fastest): solid cells get kSolid, fluid
// cells on the tunnel faces or next to a solid get kNearBoundary.
std::vector<std::uint8_t> buildFlags(const std::vector<std::uint8_t>& solid, const glm::ivec3& dims);

// The lattice flags for a body: voxelised at lattice resolution, with the
// floor gaps closed when the ground moves, then buildFlags().
std::vector<std::uint8_t> latticeFlags(const core::SurfaceMesh& body, const LbmPlan& plan);

// Map time-averaged lattice fields onto the output grid. `sums` holds the
// per-lattice-cell sums of ux, uy, uz and rho - 1 (structure of arrays, each
// `lattice` cells long) over `samples` samples. Each output cell averages the
// fluid lattice cells inside it (refine^3 of them). Output cells already
// marked solid, or with no fluid lattice cell, end up solid with zero
// velocity and pressure. Velocity is converted to m/s and pressure to
// kinematic gauge pressure (0 at the outlet).
void resampleMeans(const std::vector<float>& sums, int samples, const std::vector<std::uint8_t>& latticeFlags,
                   const glm::ivec3& lattice, int refine, const LatticeUnits& units, core::FlowField& field);

} // namespace solvers::lbm
