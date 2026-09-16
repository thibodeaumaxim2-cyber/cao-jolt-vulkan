"""Export timing curves from the bundled Mesh2Motion CC0 walk in Blender.

Run with Blender, not ordinary Python:
  blender --background assets/motion/mesh2motion-human-walk-large.blend \
    --python tools/extract_mesh2motion_walk.py -- assets/motion/mesh2motion_walk.json

The export intentionally contains only joint-rotation samples and asset
provenance.  It does not redistribute the source mesh or bake a replacement
animation.  The trainer retargets these curves to H1's bounded gait targets.
"""
import json
import math
import sys
from pathlib import Path

import bpy


ALIASES = {
    "left_thigh": ("thigh_l", "DRV_thigh_L", "CTRL_Thigh_L"),
    "right_thigh": ("thigh_r", "DRV_thigh_R", "CTRL_Thigh_R"),
    "left_calf": ("calf_l", "DRV_calf_L", "CTRL_Calf_L"),
    "right_calf": ("calf_r", "DRV_calf_R", "CTRL_Calf_R"),
    "left_foot": ("foot_l", "DRV_foot_L", "CTRL_Foot_L"),
    "right_foot": ("foot_r", "DRV_foot_R", "CTRL_Foot_R"),
    "left_arm": ("upperarm_l", "DRV_upper_arm_L", "CTRL_Arm_L"),
    "right_arm": ("upperarm_r", "DRV_upper_arm_R", "CTRL_Arm_R"),
}


def output_path() -> Path:
    marker = sys.argv.index("--") if "--" in sys.argv else -1
    return Path(sys.argv[marker + 1]) if marker >= 0 and len(sys.argv) > marker + 1 else Path("assets/motion/mesh2motion_walk.json")


def pick_bones(armature):
    result = {}
    for semantic, names in ALIASES.items():
        bone = next((armature.pose.bones.get(name) for name in names if armature.pose.bones.get(name)), None)
        if bone is None:
            raise RuntimeError(f"Cannot find a {semantic} bone; tried {names}")
        result[semantic] = bone.name
    return result


def main():
    scene = bpy.context.scene
    armature = next((obj for obj in scene.objects if obj.type == "ARMATURE"), None)
    if armature is None:
        raise RuntimeError("No armature found in the Mesh2Motion file")
    bones = pick_bones(armature)
    # 240 samples preserves timing while keeping the repository dataset tiny.
    action = armature.animation_data.action if armature.animation_data else None
    if action is None:
        raise RuntimeError("The armature has no active animation action")
    first, last = (int(value) for value in action.frame_range)
    if last <= first:
        raise RuntimeError("Animation has no usable frame interval")
    count = min(240, max(2, last - first + 1))
    frame_numbers = [round(first + index * (last - first) / (count - 1)) for index in range(count)]
    frames = []
    for frame in frame_numbers:
        scene.frame_set(frame)
        rotations = {}
        for semantic, name in bones.items():
            euler = armature.pose.bones[name].matrix.to_euler("XYZ")
            rotations[semantic] = [round(float(value), 7) for value in euler]
        frames.append({"frame": frame, "rotations_xyz_rad": rotations})
    payload = {
        "format": "cao_mesh2motion_walk_timing_v1",
        "source": f"Mesh2Motion CC0 walk / {Path(bpy.data.filepath).name or 'mesh2motion-walk.fbx'}",
        "fps": scene.render.fps / scene.render.fps_base,
        "bones": bones,
        "frames": frames,
    }
    target = output_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Exported {len(frames)} samples to {target}")


if __name__ == "__main__":
    main()
