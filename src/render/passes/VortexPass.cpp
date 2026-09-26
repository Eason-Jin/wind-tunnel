#include "render/passes/VortexPass.h"

#include "render/Colormap.h"

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <thread>

namespace render {

namespace {

// --- Standard marching cubes tables (Lorensen & Cline / Bloyd), as published
// by Paul Bourke, http://paulbourke.net/geometry/polygonise/. Cube corners
// and edges follow that page's numbering:
//
//   corner i -> grid offset: 0:(0,0,0) 1:(1,0,0) 2:(1,1,0) 3:(0,1,0)
//                            4:(0,0,1) 5:(1,0,1) 6:(1,1,1) 7:(0,1,1)
//   edge e -> corner pair:   0:(0,1) 1:(1,2) 2:(2,3) 3:(3,0)
//                            4:(4,5) 5:(5,6) 6:(6,7) 7:(7,4)
//                            8:(0,4) 9:(1,5) 10:(2,6) 11:(3,7)
constexpr glm::ivec3 kMcCornerOffset[8] = {
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
};
constexpr int kMcEdgeCorner[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
};

static constexpr int kMcEdgeTable[256] = {
    0, 265, 515, 778, 1030, 1295, 1541, 1804,
    2060, 2309, 2575, 2822, 3082, 3331, 3593, 3840,
    400, 153, 915, 666, 1430, 1183, 1941, 1692,
    2460, 2197, 2975, 2710, 3482, 3219, 3993, 3728,
    560, 825, 51, 314, 1590, 1855, 1077, 1340,
    2620, 2869, 2111, 2358, 3642, 3891, 3129, 3376,
    928, 681, 419, 170, 1958, 1711, 1445, 1196,
    2988, 2725, 2479, 2214, 4010, 3747, 3497, 3232,
    1120, 1385, 1635, 1898, 102, 367, 613, 876,
    3180, 3429, 3695, 3942, 2154, 2403, 2665, 2912,
    1520, 1273, 2035, 1786, 502, 255, 1013, 764,
    3580, 3317, 4095, 3830, 2554, 2291, 3065, 2800,
    1616, 1881, 1107, 1370, 598, 863, 85, 348,
    3676, 3925, 3167, 3414, 2650, 2899, 2137, 2384,
    1984, 1737, 1475, 1226, 966, 719, 453, 204,
    4044, 3781, 3535, 3270, 3018, 2755, 2505, 2240,
    2240, 2505, 2755, 3018, 3270, 3535, 3781, 4044,
    204, 453, 719, 966, 1226, 1475, 1737, 1984,
    2384, 2137, 2899, 2650, 3414, 3167, 3925, 3676,
    348, 85, 863, 598, 1370, 1107, 1881, 1616,
    2800, 3065, 2291, 2554, 3830, 4095, 3317, 3580,
    764, 1013, 255, 502, 1786, 2035, 1273, 1520,
    2912, 2665, 2403, 2154, 3942, 3695, 3429, 3180,
    876, 613, 367, 102, 1898, 1635, 1385, 1120,
    3232, 3497, 3747, 4010, 2214, 2479, 2725, 2988,
    1196, 1445, 1711, 1958, 170, 419, 681, 928,
    3376, 3129, 3891, 3642, 2358, 2111, 2869, 2620,
    1340, 1077, 1855, 1590, 314, 51, 825, 560,
    3728, 3993, 3219, 3482, 2710, 2975, 2197, 2460,
    1692, 1941, 1183, 1430, 666, 915, 153, 400,
    3840, 3593, 3331, 3082, 2822, 2575, 2309, 2060,
    1804, 1541, 1295, 1030, 778, 515, 265, 0,
};

static constexpr std::int8_t kMcTriTable[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};

// Raw isosurface vertex before the final colour is baked in (needs a second
// pass over the whole mesh to auto-range the colour scale).
struct RawVertex {
    glm::vec3 pos{0.0f};
    glm::vec3 normal{0.0f};
    float scalar = 0.0f; // swirl (omega_x) or speed, depending on colour mode
};

// Runs body(begin, end) over [0, n) split across hardware threads.
template <typename Fn>
void parallelFor(std::size_t n, Fn&& body)
{
    if (n == 0)
        return;
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const unsigned nThreads = static_cast<unsigned>(std::min<std::size_t>(hw, n));
    if (nThreads <= 1) {
        body(std::size_t{0}, n);
        return;
    }
    std::vector<std::thread> pool;
    pool.reserve(nThreads);
    const std::size_t chunk = (n + nThreads - 1) / nThreads;
    for (unsigned t = 0; t < nThreads; ++t) {
        const std::size_t begin = std::min(static_cast<std::size_t>(t) * chunk, n);
        const std::size_t end = std::min(begin + chunk, n);
        if (begin >= end)
            continue;
        pool.emplace_back(body, begin, end);
    }
    for (auto& th : pool)
        th.join();
}

} // namespace

VortexPass::VortexPass()
    : shader_(gl::Shader::fromFiles("vortex.vert", "vortex.frag"))
{
    threshold_ = std::pow(10.0f, thresholdLog_);

    vao_ = gl::createVertexArray();
    vbo_ = gl::createBuffer();

    glVertexArrayVertexBuffer(vao_.id(), 0, vbo_.id(), 0, sizeof(Vertex));
    glEnableVertexArrayAttrib(vao_.id(), 0);
    glVertexArrayAttribFormat(vao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, pos));
    glVertexArrayAttribBinding(vao_.id(), 0, 0);
    glEnableVertexArrayAttrib(vao_.id(), 1);
    glVertexArrayAttribFormat(vao_.id(), 1, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, normal));
    glVertexArrayAttribBinding(vao_.id(), 1, 0);
    glEnableVertexArrayAttrib(vao_.id(), 2);
    glVertexArrayAttribFormat(vao_.id(), 2, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, color));
    glVertexArrayAttribBinding(vao_.id(), 2, 0);
}

VortexPass::~VortexPass() = default;

void VortexPass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    if (haveField_) {
        computeQCriterion();
        remesh();
        lastRemeshTime_ = 0.0f;
        meshDirty_ = false;
    }
}

void VortexPass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    computeQCriterion();
    remesh();
    lastRemeshTime_ = 0.0f;
    meshDirty_ = false;
}

void VortexPass::update(const FrameContext& frame)
{
    if (meshDirty_ && haveField_ && (frame.time - lastRemeshTime_) >= kDebounceSeconds) {
        remesh();
        meshDirty_ = false;
        lastRemeshTime_ = frame.time;
    }
}

// --- Q-criterion -------------------------------------------------------------

void VortexPass::computeQCriterion()
{
    const auto t0 = std::chrono::steady_clock::now();

    qStar_.clear();
    gradQ_.clear();
    omegaXNorm_.clear();
    speedNorm_.clear();
    wallDist_.clear();
    haveField_ = false;

    if (!scene_.field || scene_.field->empty())
        return;

    const core::FlowField& field = *scene_.field;
    dims_ = field.dims;
    origin_ = field.origin;
    spacing_ = field.spacing;
    if (dims_.x < 2 || dims_.y < 2 || dims_.z < 2)
        return; // too thin to form even one marching-cubes cell

    const std::size_t n = field.cellCount();
    qStar_.assign(n, -1e9f);
    gradQ_.assign(n, glm::vec3(0.0f));
    omegaXNorm_.assign(n, 0.0f);
    speedNorm_.assign(n, 0.0f);
    wallDist_.assign(n, kMaxWallDist);

    // Characteristic length L: streamwise body length, or the field's y-z
    // extent when there's no body to measure.
    float L;
    if (scene_.body && !scene_.body->empty()) {
        L = std::max(scene_.body->bounds().size().x, 1e-4f);
    } else {
        const core::Bounds fb = field.bounds();
        L = std::max(0.5f * (fb.size().y + fb.size().z), 1e-4f);
    }
    const float U = std::max(field.freestreamSpeed, 1e-4f);
    qScale_ = (U / L) * (U / L);
    const float vortScale = U / L;

    const int nx = dims_.x, ny = dims_.y, nz = dims_.z;
    const auto index = [&](int i, int j, int k) { return field.index(i, j, k); };

    // --- Wall distance: multi-source BFS from solid cells, 6-connected. -----
    if (!field.solid.empty()) {
        std::queue<std::size_t> q;
        for (std::size_t c = 0; c < n; ++c) {
            if (field.solid[c]) {
                wallDist_[c] = 0;
                q.push(c);
            }
        }
        const int di[6] = {-1, 1, 0, 0, 0, 0};
        const int dj[6] = {0, 0, -1, 1, 0, 0};
        const int dk[6] = {0, 0, 0, 0, -1, 1};
        while (!q.empty()) {
            const std::size_t c = q.front();
            q.pop();
            const std::uint8_t d = wallDist_[c];
            if (d >= kMaxWallDist)
                continue;
            const int k = static_cast<int>(c / (static_cast<std::size_t>(nx) * ny));
            const std::size_t rem = c - static_cast<std::size_t>(k) * nx * ny;
            const int j = static_cast<int>(rem / nx);
            const int i = static_cast<int>(rem - static_cast<std::size_t>(j) * nx);
            for (int s = 0; s < 6; ++s) {
                const int ni = i + di[s], nj = j + dj[s], nk = k + dk[s];
                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz)
                    continue;
                const std::size_t nc = index(ni, nj, nk);
                if (wallDist_[nc] > d + 1) {
                    wallDist_[nc] = static_cast<std::uint8_t>(d + 1);
                    q.push(nc);
                }
            }
        }
    } else {
        std::fill(wallDist_.begin(), wallDist_.end(), kMaxWallDist);
    }

    // --- Velocity gradient -> Q, streamwise vorticity, speed. ---------------
    // One-sided differences at the domain edges, central elsewhere; solid /
    // near-wall cells are excluded afterwards regardless of what the raw
    // gradient came out to (their neighbours may include no-slip zeros that
    // would otherwise read as a spurious high-shear skin).
    const auto velAt = [&](int i, int j, int k) { return field.velocity[index(i, j, k)]; };
    const auto diffAxis = [&](int i, int j, int k, int axis) -> glm::vec3 {
        int lo[3] = {i, j, k};
        int hi[3] = {i, j, k};
        const int dimN = axis == 0 ? nx : (axis == 1 ? ny : nz);
        if (dimN < 2)
            return glm::vec3(0.0f);
        lo[axis] = std::max(lo[axis] - 1, 0);
        hi[axis] = std::min(hi[axis] + 1, dimN - 1);
        const float h = static_cast<float>(hi[axis] - lo[axis]) * spacing_[axis];
        if (h <= 1e-8f)
            return glm::vec3(0.0f);
        return (velAt(hi[0], hi[1], hi[2]) - velAt(lo[0], lo[1], lo[2])) / h;
    };

    parallelFor(n, [&](std::size_t begin, std::size_t end) {
        for (std::size_t c = begin; c < end; ++c) {
            const int k = static_cast<int>(c / (static_cast<std::size_t>(nx) * ny));
            const std::size_t rem = c - static_cast<std::size_t>(k) * nx * ny;
            const int j = static_cast<int>(rem / nx);
            const int i = static_cast<int>(rem - static_cast<std::size_t>(j) * nx);

            const glm::vec3 dVdx = diffAxis(i, j, k, 0); // (du/dx, dv/dx, dw/dx)
            const glm::vec3 dVdy = diffAxis(i, j, k, 1); // (du/dy, dv/dy, dw/dy)
            const glm::vec3 dVdz = diffAxis(i, j, k, 2); // (du/dz, dv/dz, dw/dz)

            const float J[3][3] = {
                {dVdx.x, dVdy.x, dVdz.x},
                {dVdx.y, dVdy.y, dVdz.y},
                {dVdx.z, dVdy.z, dVdz.z},
            };

            float sumS2 = 0.0f;
            float sumO2 = 0.0f;
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b) {
                    const float s = 0.5f * (J[a][b] + J[b][a]);
                    const float o = 0.5f * (J[a][b] - J[b][a]);
                    sumS2 += s * s;
                    sumO2 += o * o;
                }
            }
            const float Q = 0.5f * (sumO2 - sumS2);
            qStar_[c] = Q / qScale_;

            const float omegaX = J[2][1] - J[1][2]; // dw/dy - dv/dz
            omegaXNorm_[c] = omegaX / vortScale;
            speedNorm_[c] = glm::length(field.velocity[c]) / U;
        }
    });

    // Baseline exclusion: solid cells and anything touching solid.
    for (std::size_t c = 0; c < n; ++c) {
        if (wallDist_[c] <= 1)
            qStar_[c] = -1e9f;
    }

    // --- Gradient of Q, for smooth isosurface normals. -----------------------
    // Falls back to a one-sided difference (or zero) when a neighbour is
    // inside the excluded band, so the -1e9 sentinel never leaks into a
    // gradient used by a valid, rendered vertex.
    const auto qValid = [&](int i, int j, int k) { return wallDist_[index(i, j, k)] > 1; };
    const auto qAt = [&](int i, int j, int k) { return qStar_[index(i, j, k)]; };
    parallelFor(n, [&](std::size_t begin, std::size_t end) {
        for (std::size_t c = begin; c < end; ++c) {
            if (wallDist_[c] <= 1)
                continue;
            const int k = static_cast<int>(c / (static_cast<std::size_t>(nx) * ny));
            const std::size_t rem = c - static_cast<std::size_t>(k) * nx * ny;
            const int j = static_cast<int>(rem / nx);
            const int i = static_cast<int>(rem - static_cast<std::size_t>(j) * nx);

            glm::vec3 g(0.0f);
            const int idx3[3] = {i, j, k};
            const int dimN[3] = {nx, ny, nz};
            for (int axis = 0; axis < 3; ++axis) {
                int loI[3] = {i, j, k};
                int hiI[3] = {i, j, k};
                const bool hasLo = idx3[axis] > 0;
                const bool hasHi = idx3[axis] < dimN[axis] - 1;
                loI[axis] -= 1;
                hiI[axis] += 1;
                const bool loValid = hasLo && qValid(loI[0], loI[1], loI[2]);
                const bool hiValid = hasHi && qValid(hiI[0], hiI[1], hiI[2]);
                float gv = 0.0f;
                if (loValid && hiValid)
                    gv = (qAt(hiI[0], hiI[1], hiI[2]) - qAt(loI[0], loI[1], loI[2])) / (2.0f * spacing_[axis]);
                else if (hiValid)
                    gv = (qAt(hiI[0], hiI[1], hiI[2]) - qStar_[c]) / spacing_[axis];
                else if (loValid)
                    gv = (qStar_[c] - qAt(loI[0], loI[1], loI[2])) / spacing_[axis];
                g[axis] = gv;
            }
            gradQ_[c] = g;
        }
    });

    haveField_ = true;

    const auto t1 = std::chrono::steady_clock::now();
    computeTimeMs_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
}

// --- Marching cubes -----------------------------------------------------------

void VortexPass::remesh()
{
    const auto t0 = std::chrono::steady_clock::now();

    triangleCount_ = 0;
    vertexCount_ = 0;

    if (!haveField_) {
        uploadGeometry({});
        meshTimeMs_ = 0.0f;
        return;
    }

    const int nx = dims_.x, ny = dims_.y, nz = dims_.z;
    const std::uint8_t hideCut = static_cast<std::uint8_t>(std::clamp(hideNearWall_, 1, static_cast<int>(kMaxWallDist) - 1));
    const auto index = [&](int i, int j, int k) -> std::size_t {
        return static_cast<std::size_t>(i) + static_cast<std::size_t>(nx) * (static_cast<std::size_t>(j) + static_cast<std::size_t>(ny) * k);
    };

    const int nkCubes = std::max(nz - 1, 0);
    std::vector<std::vector<RawVertex>> perThread;
    {
        const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
        const unsigned nThreads = static_cast<unsigned>(std::min<int>(static_cast<int>(hw), std::max(nkCubes, 1)));
        perThread.resize(nThreads);

        auto worker = [&](unsigned t) {
            std::vector<RawVertex>& out = perThread[t];
            const int kBegin = static_cast<int>((static_cast<std::size_t>(t) * nkCubes) / nThreads);
            const int kEnd = static_cast<int>((static_cast<std::size_t>(t + 1) * nkCubes) / nThreads);
            for (int k = kBegin; k < kEnd; ++k) {
                for (int j = 0; j + 1 < ny; ++j) {
                    for (int i = 0; i + 1 < nx; ++i) {
                        float val[8];
                        glm::vec3 pos[8];
                        glm::vec3 grad[8];
                        float scal[8];
                        bool valid = true;
                        for (int c = 0; c < 8; ++c) {
                            const glm::ivec3 off = kMcCornerOffset[c];
                            const int ci = i + off.x, cj = j + off.y, ck = k + off.z;
                            const std::size_t idx = index(ci, cj, ck);
                            if (wallDist_[idx] <= hideCut) {
                                valid = false;
                                break;
                            }
                            val[c] = qStar_[idx];
                            pos[c] = origin_ + spacing_ * glm::vec3(ci, cj, ck);
                            grad[c] = gradQ_[idx];
                            scal[c] = (colorMode_ == ColorMode::SwirlDirection) ? omegaXNorm_[idx] : speedNorm_[idx];
                        }
                        if (!valid)
                            continue;

                        int cubeindex = 0;
                        for (int c = 0; c < 8; ++c)
                            if (val[c] < threshold_)
                                cubeindex |= (1 << c);
                        const int edgeBits = kMcEdgeTable[cubeindex];
                        if (edgeBits == 0)
                            continue;

                        glm::vec3 vertPos[12];
                        glm::vec3 vertNormal[12];
                        float vertScalar[12];
                        for (int e = 0; e < 12; ++e) {
                            if (!(edgeBits & (1 << e)))
                                continue;
                            const int a = kMcEdgeCorner[e][0];
                            const int b = kMcEdgeCorner[e][1];
                            const float va = val[a], vb = val[b];
                            float mu = std::abs(vb - va) < 1e-9f ? 0.5f : (threshold_ - va) / (vb - va);
                            mu = std::clamp(mu, 0.0f, 1.0f);
                            vertPos[e] = glm::mix(pos[a], pos[b], mu);
                            const glm::vec3 g = glm::mix(grad[a], grad[b], mu);
                            const float glen = glm::length(g);
                            vertNormal[e] = glen > 1e-8f ? (-g / glen) : glm::vec3(0.0f, 0.0f, 1.0f);
                            vertScalar[e] = glm::mix(scal[a], scal[b], mu);
                        }

                        const std::int8_t* tri = kMcTriTable[cubeindex];
                        for (int m = 0; tri[m] != -1; m += 3) {
                            for (int v = 0; v < 3; ++v) {
                                const int e = tri[m + v];
                                out.push_back({vertPos[e], vertNormal[e], vertScalar[e]});
                            }
                        }
                    }
                }
            }
        };

        if (perThread.size() <= 1) {
            worker(0);
        } else {
            std::vector<std::thread> pool;
            pool.reserve(perThread.size());
            for (unsigned t = 0; t < perThread.size(); ++t)
                pool.emplace_back(worker, t);
            for (auto& th : pool)
                th.join();
        }
    }

    std::vector<RawVertex> raw;
    std::size_t total = 0;
    for (const auto& v : perThread)
        total += v.size();
    raw.reserve(total);
    for (auto& v : perThread)
        raw.insert(raw.end(), v.begin(), v.end());

    // Auto-range the colour scale over the generated mesh and bake final
    // per-vertex colours.
    std::vector<Vertex> verts(raw.size());
    if (colorMode_ == ColorMode::SwirlDirection) {
        float maxAbs = 1e-6f;
        for (const auto& r : raw)
            maxAbs = std::max(maxAbs, std::abs(r.scalar));
        for (std::size_t i = 0; i < raw.size(); ++i) {
            const float t = 0.5f + 0.5f * std::clamp(raw[i].scalar / maxAbs, -1.0f, 1.0f);
            verts[i] = {raw[i].pos, raw[i].normal, colormap(ColormapKind::CoolWarm, t)};
        }
    } else {
        float maxV = 1e-6f;
        for (const auto& r : raw)
            maxV = std::max(maxV, r.scalar);
        for (std::size_t i = 0; i < raw.size(); ++i) {
            const float t = std::clamp(raw[i].scalar / maxV, 0.0f, 1.0f);
            verts[i] = {raw[i].pos, raw[i].normal, colormap(ColormapKind::Turbo, t)};
        }
    }

    triangleCount_ = static_cast<int>(verts.size() / 3);
    uploadGeometry(verts);

    const auto t1 = std::chrono::steady_clock::now();
    meshTimeMs_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
}

void VortexPass::uploadGeometry(const std::vector<Vertex>& verts)
{
    vertexCount_ = static_cast<int>(verts.size());
    if (vertexCount_ == 0) {
        glNamedBufferData(vbo_.id(), 0, nullptr, GL_DYNAMIC_DRAW);
        return;
    }
    glNamedBufferData(vbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(Vertex)), verts.data(), GL_DYNAMIC_DRAW);
}

// --- Rendering -----------------------------------------------------------------

void VortexPass::draw(const FrameContext& frame)
{
    if (vertexCount_ == 0)
        return;

    const bool useBlend = opacity_ < 0.999f;
    if (useBlend) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    shader_.use();
    shader_.set("uViewProj", frame.proj * frame.view);
    shader_.set("uEye", frame.eye);
    shader_.set("uOpacity", opacity_);

    glBindVertexArray(vao_.id());
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);

    if (useBlend) {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

void VortexPass::drawUi()
{
    if (!scene_.field || scene_.field->empty()) {
        ImGui::TextDisabled("No flow field loaded.");
        return;
    }

    int mode = static_cast<int>(colorMode_);
    if (ImGui::Combo("Colour by", &mode, "Swirl direction (omega_x)\0Velocity magnitude\0")) {
        colorMode_ = static_cast<ColorMode>(mode);
        markMeshDirty();
    }

    if (ImGui::SliderFloat("Threshold (log10 Q*)", &thresholdLog_, -2.0f, 3.0f, "%.2f")) {
        threshold_ = std::pow(10.0f, thresholdLog_);
        markMeshDirty();
    }
    ImGui::SameLine();
    ImGui::Text("(%.3g)", static_cast<double>(threshold_));

    if (ImGui::SliderInt("Hide near-wall (cells)", &hideNearWall_, 1, static_cast<int>(kMaxWallDist) - 1))
        markMeshDirty();

    ImGui::SliderFloat("Opacity", &opacity_, 0.05f, 1.0f);

    ImGui::Separator();
    ImGui::Text("%d triangles", triangleCount_);
    ImGui::Text("Q compute: %.2f ms  |  Mesh: %.2f ms", static_cast<double>(computeTimeMs_), static_cast<double>(meshTimeMs_));
}

} // namespace render
