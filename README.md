# Wind Tunnel

A virtual wind tunnel: load an STL, run CFD around it with OpenFOAM, and explore the flow with OpenGL.

## Build

Requires CMake ≥ 3.24, GCC with C++20, OpenGL 4.6, and the GLFW build dependencies:

```bash
sudo apt install cmake build-essential libwayland-dev libxkbcommon-dev xorg-dev
cmake -B build && cmake --build build -j
```

GLFW, GLM, Dear ImGui and stb are fetched automatically. OpenFOAM ESI v2406 (`/usr/lib/openfoam/openfoam2406`) is optional. It is only needed to run real simulations.

## Run

```bash
./build/windtunnel                                  # test sphere with an instant potential-flow preview
./build/windtunnel --stl car.stl --scale 0.001      # STL in millimetres
./build/windtunnel --stl car.stl --up y --yaw 90    # re-orient: flow is along +x, +z is up
./build/windtunnel --stl car.stl --solve openfoam   # start an OpenFOAM run immediately
```

In the window:
1. Load a body and orient it.
2. Pick the **OpenFOAM** solver and press **Run**. On 8 cores a car-sized run takes about 30–40 s at refinement 3, or about 5 min at refinement 4.
3. Switch views in **Display**: surface Cp, slice plane, streamlines, particles.

Results are saved in `cases/run/`. **Open existing OpenFOAM result** reloads them without solving again.

Mouse controls: left-drag to orbit, right-drag to pan, the wheel to zoom, and `F` to reframe.

`--screenshot out.png [--frames N] [--camera yaw,pitch,zoom] [--passes model,slice,...]` renders offscreen and exits. Run `--help` for all options.

## Layout

| Module | Role |
|---|---|
| `src/core` | Solver-independent data: `SurfaceMesh`, `FlowField` (uniform grid), `TunnelDomain`, `ISolver` |
| `src/io` | STL read/write |
| `src/solvers/openfoam` | Case writer, process runner, field reader. Pipeline: blockMesh → snappyHexMesh → simpleFoam (k-ω SST) → mapFields onto the output grid |
| `src/solvers/synthetic` | Analytic potential flow for instant previews |
| `src/render` | GL wrappers, camera, `FlowTextures` (3D textures), and passes: tunnel, model, slice, streamlines, particles |
| `src/app` | Window, UI, solver worker thread, screenshot mode |

The renderer only ever sees `core::FlowField`. To add a new solver (e.g. a CUDA LBM), implement `core::ISolver` and produce a field on `core::makeTunnelDomain`'s grid.
