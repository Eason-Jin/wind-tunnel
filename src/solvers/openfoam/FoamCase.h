#pragma once

#include "core/SurfaceMesh.h"
#include "core/TunnelDomain.h"

#include <glm/glm.hpp>

#include <filesystem>

namespace solvers::foam {

// Everything needed to write the simpleFoam case around a body. Built by
// makeCaseSpec() from the body bounds and SimulationParams.
struct CaseSpec {
    core::Bounds body;          // body bounds (m)
    core::TunnelDomain domain;  // output grid / tunnel box
    glm::dvec3 meshMin{0.0};    // CFD domain (== domain.box, except the floor may be raised)
    glm::dvec3 meshMax{0.0};
    glm::ivec3 backgroundCells{1};
    int surfaceLevel = 3;       // snappy level on the body
    int regionLevel = 2;        // snappy level inside the near-wake box
    glm::dvec3 refineMin{0.0};  // near-wake refinement box
    glm::dvec3 refineMax{0.0};
    glm::dvec3 locationInMesh{0.0};
    bool groundPlane = true;    // floor is a moving-ground wall
    double inletSpeed = 20.0;
    double nu = 1.5e-5;
    double k = 0.06;            // inlet turbulent kinetic energy
    double omega = 1.0;         // inlet specific dissipation
    int iterations = 400;
    int processors = 1;
};

// Sizes the background mesh, refinement and inlet turbulence for a body.
// `bodyBounds` and the params must be what makeTunnelDomain() is given.
struct CaseInputs {
    core::Bounds bodyBounds;
    double inletSpeed;
    double nu;
    bool groundPlane;
    int refinementLevel;
    int iterations;
    int processors;
};
CaseSpec makeCaseSpec(const CaseInputs& in, const core::TunnelDomain& domain);

// Write system/, constant/ (except triSurface) and 0.orig/ of the main case.
void writeMainCase(const std::filesystem::path& caseDir, const CaseSpec& spec);

// Write the mapping-target case: a single-block blockMesh identical to the
// output grid, sentinel-initialised U and p in 0/, and a mapFieldsDict.
void writeGridCase(const std::filesystem::path& gridDir, const core::TunnelDomain& domain);

// Value written into the grid case's U so unmapped cells can be detected.
constexpr float kUnmappedSentinel = 1.0e20f;

} // namespace solvers::foam
