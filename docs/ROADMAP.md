# CAO Native Roadmap

## Current foundation

- Vulkan interactive application with an editor-oriented scene model
- MuJoCo articulated dynamics and a headless executable
- Stable hexapod crawl and alternating-tripod regression checks
- Unitree H1 imported as an official MuJoCo asset set with real STL rendering
- Primitive creation, selection, transforms, pyramid demo, and JSON persistence
- CTest scene-model, equilibrium, gait, biped, and H1 import checks

## Next milestone: editor reliability

1. Add scene serialization round-trip tests and physics regression tests.
2. Add undo/redo using the existing `SceneCommand` types.
3. Version the JSON scene format and provide migration errors.
4. Document a clean Debug/Release build matrix for Linux and Windows.
5. Add a dedicated Unitree H1 controller before enabling H1 simulation.

## Renderer milestone

1. Add selection outlines and stable minimize handling.
2. Add mesh normals, materials, textures, and glTF import.
3. Add GPU/CPU timing overlays and RenderDoc capture notes.
4. Add LOD/indexed mesh optimization for large imported robot assets.

## Product milestone

1. Separate editor, scene, physics, serialization, and renderer APIs.
2. Add hierarchy, multi-selection, duplication, grouping, and prefabs.
3. Add packaged example scenes and a short recorded demonstration.
4. Publish a tagged developer release once the headless checks are reproducible.

## Design rule

Keep the scene/editor model independent from Vulkan and MuJoCo handles. Rendering and physics should consume synchronized scene data, not own the editor's source of truth.
