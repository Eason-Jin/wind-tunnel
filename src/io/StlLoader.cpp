#include "io/StlLoader.h"

#include <stdexcept>

namespace io {

// TODO: stub, to be implemented.
core::SurfaceMesh loadStl(const std::filesystem::path& path)
{
    throw std::runtime_error("loadStl not implemented yet: " + path.string());
}

void writeStl(const core::SurfaceMesh&, const std::filesystem::path& path, const std::string&)
{
    throw std::runtime_error("writeStl not implemented yet: " + path.string());
}

} // namespace io
