# CAO MuJoCo Vulkan

Standalone C++20/Vulkan replacement for the portfolio CAO 3D physics app. It has no PHP, browser, CDN, Three.js, or Rapier dependency.

## Features

- MuJoCo articulated dynamics, contact solving, position actuators, friction, and a static ground plane.
- Six-leg robot with 24 rotary actuators and a balance-gated one-foot crawl gait.
- Official Unitree H1 MuJoCo model import, including the upstream STL assets and inertial data.
- Fast equilibrium policy that pauses walking on instability and resumes only after recovery.
- Box, cylinder, sphere and beam creation.
- Mouse selection, object tree, editable transform/property panel, translate/rotate/scale modes, delete and snap-to-0.25m.
- Perspective orbit camera, ISO/top/front/right presets and frame-all.
- Configurable 2–12-level cube pyramid, paused Build and falling Demo.
- Space/play/pause, automatic demo restart after the pyramid falls 50m or the world is inactive for 7 seconds.
- JSON scene export/import and New scene.
- Minimal dependency footprint: GLFW + Vulkan + Dear ImGui + nlohmann/json + MuJoCo.

## Build

Install CMake 3.20+, a C++20 compiler, Vulkan development files (including `glslc`), and GLFW prerequisites. MuJoCo, GLFW, Dear ImGui, and nlohmann/json are fetched and compiled automatically by default. Then:

```bash
cmake -S . -B build
cmake --build build --config Release -j
./build/cao-jolt
```

The H1 model is an upstream Git submodule. Initialize it after a fresh clone:

```bash
git submodule update --init --recursive
```

It is sourced from Unitree's BSD-3-Clause-licensed
[`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) repository.

The first configure downloads the pinned MuJoCo release from its official repository and builds it as part of this project.

On Windows, use the generated `Release/cao-jolt.exe`. If the machine is offline, pre-populate CMake's FetchContent cache or configure on a machine with network access first.

Run a short deterministic gait check after building:

```bash
./build/cao-headless walk --quick
```

Omit `--quick` to run the complete 10-candidate, 60-second controller search.

## Controls

Left click selects. Right drag orbits. Middle drag pans. Wheel zooms. `W`, `E`, `R` select transform modes. `Space` toggles simulation. `Delete` removes the selection. `N` creates a box. `Ctrl+S` exports `scene.json`; `Ctrl+O` imports it. The on-screen toolbar documents the remaining actions.

## MuJoCo integration

`src/editor/MuJoCoBridge.cpp` creates an in-memory MJCF hexapod with a free torso, 24 constrained position actuators, high-friction feet, and a ground plane. The render scene mirrors MuJoCo geometry transforms each frame while `src/renderer/VulkanRenderer.cpp` owns the Vulkan presentation path. `cao-headless` provides a windowless simulation target for regression checks.

The **Import Unitree H1** toolbar action loads Unitree's original `scene.xml`
and mesh assets into MuJoCo. Vulkan loads and renders the matching binary STL
mesh for every visible H1 link, synchronized to MuJoCo body transforms. The
H1 stays paused until a dedicated balance/controller layer is implemented.

## Project status

The project is an active prototype moving toward a reusable native physics editor. The Vulkan path is the supported interactive renderer; the old OpenGL wording has been removed because there is currently no OpenGL application entry point in this repository.

See [docs/ROADMAP.md](docs/ROADMAP.md) for the recommended development sequence.
