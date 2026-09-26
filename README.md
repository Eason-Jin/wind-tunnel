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
1. Load a model with **Open STL…** (native file dialog) or **Samples** (the OpenFOAM tutorial models).
2. Set the file units and orientation in the left panel. Wind blows along +X, so the nose should point towards -X.
3. Set **Wind speed** (km/h or m/s) and press **Simulate / Play** (`Space`).
   - With **OpenFOAM CFD** selected, this runs the simulation first. A car-sized model takes about 40 s at Normal quality on 8 cores.
   - It then animates the flow: smoke, marching streamlines and vortex cores.
4. Toggle layers on the right:
   - body surface pressure (Cp)
   - streamlines
   - smoke
   - vortex cores
   - **Section plane**: drag its position slider to re-trace in-plane streamlines live. The model is cut away at the plane.

Use the **view cube** (top right) to snap to faces, edges or corners.

Results are saved in `cases/run/`, and **Advanced → Open saved result** reloads them.

Mouse controls: left-drag to orbit, right-drag to pan, the wheel to zoom, and `F` to reframe.

`--screenshot out.png [--frames N] [--camera yaw,pitch,zoom] [--passes model,slice,...] [--ui] [--play]` renders offscreen and exits. Run `--help` for all options.

## Game models (BeamNG)

Game meshes are loose, open panels. The app handles them directly: the solid is found by flood-filling the air from outside, so gaps narrower than a grid cell don't leak.

```bash
python3 tools/beamng_to_stl.py MOD.zip --list                          # vehicles and configurations
python3 tools/beamng_to_stl.py MOD.zip --config facelift_gxl -o car.stl
./build/windtunnel --stl car.stl
```

- `beamng_to_stl.py` resolves the chosen configuration's parts.
- It places tyres at the jbeam wheel-hub nodes (track, wheelbase and ride height come from the vehicle).
- It rotates the car so the nose faces the wind. Needs `numpy`.
- Optional: `./build/wt_prep in.stl out.stl` shrink-wraps a mesh into one smooth closed hull. It seals grilles and panel gaps, but rounds off detail.

## Layout

| Module | Role |
|---|---|
| `src/core` | Solver-independent data: `SurfaceMesh`, `FlowField` (uniform grid), `TunnelDomain`, `ISolver` |
| `src/io` | STL read/write |
| `src/solvers/openfoam` | Case writer, process runner, field reader. Pipeline: blockMesh → snappyHexMesh → simpleFoam (k-ω SST) → mapFields onto the output grid |
| `src/solvers/synthetic` | Analytic potential flow for instant previews |
| `src/render` | GL wrappers, camera, `FlowTextures` (3D textures), and passes: tunnel, model, slice (with in-plane streamlines), streamlines, particles, vortex cores |
| `src/app` | Window, slicer-style UI (`AppUi.cpp`, `ui/`: theme, view cube, file picker), solver worker thread, screenshot mode |

The renderer only ever sees `core::FlowField`. To add a new solver (e.g. a CUDA LBM), implement `core::ISolver` and produce a field on `core::makeTunnelDomain`'s grid.
