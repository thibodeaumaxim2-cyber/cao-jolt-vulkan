# Native CAO parity target

This project replaces the portfolio's browser CAO application with an independent C++20 application using Vulkan and Jolt.

## Native editor target

- Vulkan 3D viewport with perspective orbit, pan, zoom, grid and axes.
- Selectable boxes, cylinders, spheres and compression beams.
- Object tree and property inspector: name, transform, scale, dynamic/static state and material.
- Direct transform editing (move / rotate / scale) and delete selection.
- Pyramid builder: 2–12 levels, centred on a platform, with reset/rebuild.
- Simulation controls: play, pause, reset and structural-health state.
- Jolt rigid-body scene: static ground/platform and dynamic object bodies.
- Jolt-to-render transform synchronization.
- Dyneema cable objects, endpoint connections and tension/health values.
- Save/load `.cao.json` scenes.

## Delivery sequence

1. Make the native viewport show a real depth-buffered cube pyramid and workspace grid.
2. Replace the static mesh with per-object render records and selection.
3. Create/sync Jolt bodies for the pyramid and simulation controls.
4. Add the native scene tree/properties UI and persistence.
5. Add beam/cable structural analysis and visual diagnostics.

The current executable is the Vulkan viewport foundation. Features are only marked complete when they are usable in the running application, not when their data types merely exist.
