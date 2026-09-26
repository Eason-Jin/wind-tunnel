#pragma once

#include "core/SurfaceMesh.h"

#include <functional>
#include <string>

namespace core {

struct ShrinkWrapOptions {
    float voxelSize = 0.012f;  // metres (same units as the mesh); ~L/300 works well
    int closeRadius = 7;       // gaps up to ~2*closeRadius voxels wide are sealed (grilles, panel gaps)
    int smoothPasses = 3;      // box-blur passes on the occupancy before meshing
    int taubinIterations = 30; // surface smoothing after meshing (removes voxel terracing, keeps volume)
};

// Turn a "game" mesh (loose, open, overlapping panels) into one closed,
// manifold hull suitable for voxelisation and CFD meshing:
//   1. rasterise the triangles into a voxel grid,
//   2. morphologically close it so panel gaps are sealed,
//   3. flood-fill from outside; everything not reached is solid,
//   4. extract the solid's surface with marching cubes.
// Interior detail (cabin, engine) is discarded. `log` receives progress text.
SurfaceMesh shrinkWrap(const SurfaceMesh& in, const ShrinkWrapOptions& options,
                       const std::function<void(const std::string&)>& log = {});

} // namespace core
