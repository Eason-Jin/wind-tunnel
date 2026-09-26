#pragma once

#include "core/FlowField.h"
#include "core/SurfaceMesh.h"

namespace core {

// Mark field.solid[n] = 1 for every cell the body occupies: cells its surface
// passes through, plus any volume the outside air cannot reach from the grid
// boundary. Works for open, multi-part meshes (gaps narrower than a cell
// cannot leak), so raw game models need no repair. Zeroes velocity there.
void voxelizeSolid(const SurfaceMesh& body, FlowField& field);

} // namespace core
