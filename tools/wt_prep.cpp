// wt_prep: make any STL usable in the wind tunnel by shrink-wrapping it into
// a single closed hull (seals panel gaps, drops interior detail).
//
//   wt_prep input.stl output.stl [--voxel 0.012] [--close 7] [--smooth 3] [--taubin 30]

#include "core/ShrinkWrap.h"
#include "io/StlLoader.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: wt_prep input.stl output.stl [--voxel metres] [--close voxels] [--smooth passes] [--taubin iterations]\n";
        return 2;
    }
    core::ShrinkWrapOptions opt;
    for (int i = 3; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--voxel"))
            opt.voxelSize = std::stof(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--close"))
            opt.closeRadius = std::stoi(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--smooth"))
            opt.smoothPasses = std::stoi(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--taubin"))
            opt.taubinIterations = std::stoi(argv[i + 1]);
        else {
            std::cerr << "unknown option " << argv[i] << '\n';
            return 2;
        }
    }
    try {
        const auto t0 = std::chrono::steady_clock::now();
        const core::SurfaceMesh in = io::loadStl(argv[1]);
        std::cout << "input: " << in.triangleCount() << " triangles\n";
        const core::SurfaceMesh hull = core::shrinkWrap(in, opt, [](const std::string& s) { std::cout << "  " << s << '\n'; });
        io::writeStlBinary(hull, argv[2]);
        const auto secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const glm::vec3 s = hull.bounds().size();
        std::printf("wrote %s: %zu triangles, %.3f x %.3f x %.3f m, %.1f s\n", argv[2], hull.triangleCount(), s.x, s.y, s.z, secs);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
