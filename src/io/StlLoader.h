#pragma once

#include "core/SurfaceMesh.h"

#include <filesystem>

namespace io {

// Load an STL file (ASCII or binary, auto-detected; ASCII files may contain
// several solids). Normals are recomputed from the triangle winding.
// Throws std::runtime_error on unreadable or malformed files.
core::SurfaceMesh loadStl(const std::filesystem::path& path);

// Write a mesh as ASCII STL with a single solid named `solidName`.
// Throws std::runtime_error if the file cannot be written.
void writeStl(const core::SurfaceMesh& mesh, const std::filesystem::path& path, const std::string& solidName = "body");

} // namespace io
