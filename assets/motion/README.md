# Mesh2Motion human walk source

`mesh2motion-human-walk-large.blend` is imported from
[`Mesh2Motion/mesh2motion-assets`](https://github.com/Mesh2Motion/mesh2motion-assets),
path `rigs/human/animation-human-walk-large.blend`, revision cloned on
2026-09-16. It is a human large-walk animation used only as motion-imitation
training source for the H1 policy.

Mesh2Motion releases its rigs and animations under CC0 1.0. The upstream
license is retained as `Mesh2Motion-CC0-LICENSE.txt`.

`mesh2motion_walk.json` is generated locally from the `.blend` file by
`tools/extract_mesh2motion_walk.py`. It contains sampled joint timing only;
`tools/train_h1_policy.py` consumes it to retarget a bounded H1 hip, knee, and
ankle policy. Regenerate it after replacing the source animation:

```bash
blender --background assets/motion/mesh2motion-human-walk-large.blend \
  --python tools/extract_mesh2motion_walk.py -- assets/motion/mesh2motion_walk.json
```

`mesh2motion-walk.fbx` is an animated FBX exported from the same CC0 source.
It is the portable interchange source for the H1 retargeting pipeline. To
recreate it, run `tools/export_mesh2motion_fbx.py` through Blender, then run
the extractor against the FBX (not the `.blend`):

```bash
blender --background assets/motion/mesh2motion-human-walk-large.blend \
  --python tools/export_mesh2motion_fbx.py -- assets/motion/mesh2motion-walk.fbx
blender --background --python-expr "import bpy; bpy.ops.import_scene.fbx(filepath='assets/motion/mesh2motion-walk.fbx')" \
  --python tools/extract_mesh2motion_walk.py -- assets/motion/mesh2motion_walk.json
```
