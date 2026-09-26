#include "core/TunnelDomain.h"

#include <algorithm>
#include <cmath>

namespace core {

TunnelDomain makeTunnelDomain(const Bounds& body, const SimulationParams& params)
{
    const glm::vec3 size = body.size();
    const float L = std::max({size.x, size.y, size.z, 1e-6f});

    TunnelDomain d;
    d.box.min = {body.min.x - params.upstream * L, body.min.y - params.side * L,
                 params.groundPlane ? body.min.z : body.min.z - params.side * L};
    d.box.max = {body.max.x + params.downstream * L, body.max.y + params.side * L, body.max.z + params.side * L};

    const glm::vec3 extent = d.box.size();
    const int nx = std::max(params.gridCellsX, 8);
    d.cellSize = extent.x / static_cast<float>(nx);
    d.cells = {nx, std::max(1, static_cast<int>(std::lround(extent.y / d.cellSize))),
               std::max(1, static_cast<int>(std::lround(extent.z / d.cellSize)))};
    // Stretch y/z so the grid exactly spans the box (cells stay near-cubic).
    d.box.max.y = d.box.min.y + d.cellSize * static_cast<float>(d.cells.y);
    d.box.max.z = d.box.min.z + d.cellSize * static_cast<float>(d.cells.z);
    return d;
}

FlowField makeFieldForDomain(const TunnelDomain& domain)
{
    FlowField f;
    f.spacing = glm::vec3(domain.cellSize);
    f.origin = domain.box.min + 0.5f * f.spacing;
    f.resize(domain.cells);
    return f;
}

} // namespace core
