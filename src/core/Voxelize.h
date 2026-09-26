#pragma once

#include "core/FlowField.h"
#include "core/SurfaceMesh.h"

namespace core {

// Mark field.solid[n] = 1 for every cell centre inside the (closed) body mesh,
// using ray parity along +x. Also zeroes velocity in solid cells.
// Tolerates small holes: a column with an odd number of crossings is ignored.
void voxelizeSolid(const SurfaceMesh& body, FlowField& field);

} // namespace core
