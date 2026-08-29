# CAO Jolt Native

Standalone C++ replacement for the portfolio CAO 3D physics app. It has no PHP, browser, CDN, Three.js, or Rapier dependency.

## Features

- Jolt Physics rigid bodies, gravity, contacts, friction, restitution, sleeping and a static ground plane.
- Box, cylinder, sphere and beam creation.
- Mouse selection, object tree, editable transform/property panel, translate/rotate/scale modes, delete and snap-to-0.25m.
- Perspective orbit camera, ISO/top/front/right presets and frame-all.
- Configurable 2–12-level cube pyramid, paused Build and falling Demo.
- Space/play/pause, automatic demo restart after the pyramid falls 50m or the world is inactive for 7 seconds.
- JSON scene export/import and New scene.
- Minimal dependency footprint: GLFW + OpenGL + nlohmann/json + compiled Jolt library.

## Build

Install CMake, a C++20 compiler, and OpenGL development files. Jolt, GLFW, and nlohmann/json are fetched and compiled automatically by default. Then:

```bash
cmake -S . -B build
cmake --build build --config Release -j
./build/cao-jolt
```

The first configure downloads Jolt Physics from its official repository and builds it as part of this project. Use `-DJOLT_GIT_TAG=v5.6.0` to select another Jolt release.

If Jolt is only a raw header plus library pair:

```bash
cmake -S . -B build \
  -DJOLT_INCLUDE_DIR=/path/to/jolt/include \
  -DJOLT_LIBRARY=/path/to/libJolt.a
cmake --build build --config Release -j
```

On Windows, use `-DJOLT_LIBRARY=C:/path/to/Jolt.lib`. If the machine is offline, use `-DCAO_FETCH_JOLT=OFF -DJOLT_ROOT=C:/path/to/jolt` and install GLFW and nlohmann/json, or configure with `-DCAO_FETCH_DEPENDENCIES=OFF` when those two libraries are already installed.

On Windows, use the generated `Release/cao-jolt.exe`. The application expects the Jolt CMake target to be named `Jolt::Jolt`; if your package exports another name, change one line in `CMakeLists.txt`.

## Controls

Left click selects. Right drag orbits. Middle drag pans. Wheel zooms. `W`, `E`, `R` select transform modes. `Space` toggles simulation. `Delete` removes the selection. `N` creates a box. `Ctrl+S` exports `scene.json`; `Ctrl+O` imports it. The on-screen toolbar documents the remaining actions.

## Jolt integration

`src/main.cpp` creates a `JPH::PhysicsSystem`, broad phase, object layer filters, body interface, shape settings and dynamic/static bodies. The render scene mirrors Jolt transforms each frame. Replace the placeholder OpenGL drawing backend with your preferred renderer without changing the physics/editor model.
