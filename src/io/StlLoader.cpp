#include "io/StlLoader.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace io {

namespace {

    // Read binary STL file
    core::SurfaceMesh loadStlBinary(const std::filesystem::path& path, std::size_t fileSize)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
            throw std::runtime_error("Cannot open STL file: " + path.string());

        // Read 80-byte header
        char header[80];
        if (!file.read(header, 80) || file.gcount() != 80)
            throw std::runtime_error("Truncated binary STL file: cannot read header");

        // Read triangle count at offset 80
        uint32_t triangleCount = 0;
        if (!file.read(reinterpret_cast<char*>(&triangleCount), 4) || file.gcount() != 4)
            throw std::runtime_error("Truncated binary STL file: cannot read triangle count");

        // Validate expected file size: 84 + 50*n
        std::size_t expectedSize = 84 + static_cast<std::size_t>(triangleCount) * 50;
        if (fileSize != expectedSize)
            throw std::runtime_error("Truncated binary STL file: expected " + std::to_string(expectedSize) +
                                   " bytes, got " + std::to_string(fileSize));

        if (triangleCount == 0)
            throw std::runtime_error("STL file contains zero triangles");

        core::SurfaceMesh mesh;
        mesh.name = path.stem().string();
        mesh.positions.reserve(triangleCount * 3);

        // Read each triangle: 12 floats (normal + v1, v2, v3) + uint16 attribute
        for (uint32_t t = 0; t < triangleCount; ++t) {
            float normal[3], v1[3], v2[3], v3[3];
            uint16_t attribute = 0;

            // Read normal (3 floats = 12 bytes)
            if (!file.read(reinterpret_cast<char*>(normal), 12) || file.gcount() != 12)
                throw std::runtime_error("Truncated binary STL file: cannot read triangle normal");

            // Read vertices
            if (!file.read(reinterpret_cast<char*>(v1), 12) || file.gcount() != 12)
                throw std::runtime_error("Truncated binary STL file: cannot read vertex 1");
            if (!file.read(reinterpret_cast<char*>(v2), 12) || file.gcount() != 12)
                throw std::runtime_error("Truncated binary STL file: cannot read vertex 2");
            if (!file.read(reinterpret_cast<char*>(v3), 12) || file.gcount() != 12)
                throw std::runtime_error("Truncated binary STL file: cannot read vertex 3");

            // Read attribute count
            if (!file.read(reinterpret_cast<char*>(&attribute), 2) || file.gcount() != 2)
                throw std::runtime_error("Truncated binary STL file: cannot read attribute");

            mesh.positions.push_back(glm::vec3(v1[0], v1[1], v1[2]));
            mesh.positions.push_back(glm::vec3(v2[0], v2[1], v2[2]));
            mesh.positions.push_back(glm::vec3(v3[0], v3[1], v3[2]));
        }

        mesh.computeFaceNormals();
        return mesh;
    }

    // Read ASCII STL file
    core::SurfaceMesh loadStlAscii(const std::filesystem::path& path)
    {
        std::ifstream file(path);
        if (!file)
            throw std::runtime_error("Cannot open STL file: " + path.string());

        core::SurfaceMesh mesh;
        bool nameSet = false;

        std::string line;
        std::size_t vertexCount = 0;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;

            if (!(iss >> token))
                continue;

            if (token == "solid") {
                // Read solid name if not already set
                if (!nameSet) {
                    mesh.name.clear();
                    while (iss >> token) {
                        if (!mesh.name.empty())
                            mesh.name += " ";
                        mesh.name += token;
                    }
                    if (mesh.name.empty())
                        mesh.name = path.stem().string();
                    nameSet = true;
                }
            } else if (token == "vertex") {
                float x, y, z;
                if (!(iss >> x >> y >> z)) {
                    throw std::runtime_error("Malformed vertex in ASCII STL file: " + path.string());
                }
                mesh.positions.push_back(glm::vec3(x, y, z));
                ++vertexCount;
            }
        }

        if (vertexCount == 0)
            throw std::runtime_error("STL file contains zero triangles");

        if (vertexCount % 3 != 0)
            throw std::runtime_error("ASCII STL file has " + std::to_string(vertexCount) +
                                   " vertices (not a multiple of 3)");

        mesh.computeFaceNormals();
        return mesh;
    }

} // namespace

core::SurfaceMesh loadStl(const std::filesystem::path& path)
{
    // Check if file exists and get file size
    std::error_code ec;
    std::size_t fileSize = std::filesystem::file_size(path, ec);
    if (ec)
        throw std::runtime_error("Cannot open or read STL file: " + path.string());

    // Auto-detect: binary iff the header's triangle count matches the file size
    // (binary files may also start with "solid", so that keyword alone decides nothing).
    char head[84] = {};
    {
        std::ifstream probe(path, std::ios::binary);
        if (!probe)
            throw std::runtime_error("Cannot open or read STL file: " + path.string());
        probe.read(head, sizeof head);
    }
    if (fileSize >= 84) {
        uint32_t count = 0;
        std::memcpy(&count, head + 80, sizeof count);
        if (fileSize == 84 + static_cast<std::size_t>(count) * 50)
            return loadStlBinary(path, fileSize);
    }

    const std::string start(head, std::min<std::size_t>(fileSize, sizeof head));
    const auto first = start.find_first_not_of(" \t\r\n");
    if (first != std::string::npos && start.compare(first, 5, "solid") == 0)
        return loadStlAscii(path);

    throw std::runtime_error("Not a valid STL file (truncated binary or unknown format): " + path.string());
}

} // namespace io
