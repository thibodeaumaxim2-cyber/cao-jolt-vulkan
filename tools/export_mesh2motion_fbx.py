"""Export the bundled CC0 Mesh2Motion walk as a portable animated FBX.

Run via Blender:
  blender --background assets/motion/mesh2motion-human-walk-large.blend \
    --python tools/export_mesh2motion_fbx.py -- assets/motion/mesh2motion-walk.fbx
"""
import sys
from pathlib import Path

import bpy


def main():
    marker = sys.argv.index("--") if "--" in sys.argv else -1
    target = Path(sys.argv[marker + 1]) if marker >= 0 and len(sys.argv) > marker + 1 else Path("assets/motion/mesh2motion-walk.fbx")
    # Background Blender has no UI selection context. Exporting the scene is
    # safe here: the source file only contributes the animation-bearing rig.
    armature = next((obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"), None)
    if armature is None:
        raise RuntimeError("No animated armature found")
    target.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.fbx(filepath=str(target), use_selection=False, add_leaf_bones=False,
                             bake_anim=True, bake_anim_use_all_bones=True,
                             bake_anim_use_nla_strips=False, bake_anim_use_all_actions=False)
    print(f"Exported animated FBX: {target}")


if __name__ == "__main__":
    main()
