#pragma once

#include "core/FlowField.h"
#include "core/SurfaceMesh.h"

#include <cstdint>
#include <vector>

namespace core {

// The solid mask alone for a uniform grid of `dims` cubic-ish cells whose cell
// (0,0,0) is centred at `origin` (flat index x fastest, like FlowField). Same
// rules as voxelizeSolid(); used for grids too large to hold as a FlowField
// (e.g. a solver's own lattice).
std::vector<std::uint8_t> voxelizeSolidMask(const SurfaceMesh& body, const glm::vec3& origin, const glm::vec3& spacing,
                                            const glm::ivec3& dims);

// Mark field.solid[n] = 1 for every cell the body occupies: cells its surface
// passes through, plus any volume the outside air cannot reach from the grid
// boundary. Works for open, multi-part meshes (gaps narrower than a cell
// cannot leak), so raw game models need no repair. Zeroes velocity there.
void voxelizeSolid(const SurfaceMesh& body, FlowField& field);

} // namespace core
