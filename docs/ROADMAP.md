# CAO Native Roadmap

## Current foundation

- Vulkan interactive application with an editor-oriented scene model
- Jolt rigid-body simulation and headless executable
- Primitive creation, selection, transforms, pyramid demo, and JSON persistence
- Linux build script and CI validation
- CTest scene-model regression suite

## Next milestone: editor reliability

1. Add deterministic fixed-timestep simulation to the headless target.
2. Add scene serialization round-trip tests and physics regression tests.
3. Add undo/redo using the existing `SceneCommand` types.
4. Version the JSON scene format and provide migration errors.
5. Document a clean Debug/Release build matrix for Linux and Windows.

## Renderer milestone

1. Finish the Vulkan resource lifetime and swapchain-recreation paths.
2. Add depth, grid, selection outlines, and stable resize/minimize handling.
3. Add indexed meshes, materials, textures, and glTF import.
4. Add GPU/CPU timing overlays and RenderDoc capture notes.

## Product milestone

1. Separate editor, scene, physics, serialization, and renderer APIs.
2. Add hierarchy, multi-selection, duplication, grouping, and prefabs.
3. Add packaged example scenes and a short recorded demonstration.
4. Publish a tagged developer release once the headless checks are reproducible.

## Design rule

Keep the scene/editor model independent from Vulkan and Jolt handles. Rendering and physics should consume synchronized scene data, not own the editor's source of truth.
