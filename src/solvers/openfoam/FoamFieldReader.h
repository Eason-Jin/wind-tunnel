#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace solvers::foam {

// Read the internalField of an ASCII OpenFOAM volVectorField / volScalarField.
// Handles `uniform v`, `nonuniform List<T> N (...)` and the compact
// `nonuniform List<T> N{v}` form. `expectedCount` > 0 expands uniform values
// and checks list lengths. Throws std::runtime_error on binary or malformed
// files.
std::vector<glm::vec3> readVectorField(const std::filesystem::path& file, std::size_t expectedCount);
std::vector<float> readScalarField(const std::filesystem::path& file, std::size_t expectedCount);

} // namespace solvers::foam
