#!/usr/bin/env python3
"""Turn Mesh2Motion walk timing into a bounded, direct H1 motion reference."""
import argparse
import json
from pathlib import Path


def curve(frames, name):
    raw = [frame["rotations_xyz_rad"][name] for frame in frames]
    axis = max(zip(*raw), key=lambda values: max(values) - min(values))
    centre = sum(axis) / len(axis)
    scale = max(1e-5, max(abs(value - centre) for value in axis))
    return [(value - centre) / scale for value in axis]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion", type=Path, default=Path("assets/motion/mesh2motion_walk.json"))
    parser.add_argument("--output", type=Path, default=Path("assets/h1_motion_reference.json"))
    args = parser.parse_args()
    source = json.loads(args.motion.read_text())
    frames = source["frames"]
    left_thigh, right_thigh = curve(frames, "left_thigh"), curve(frames, "right_thigh")
    left_calf, right_calf = curve(frames, "left_calf"), curve(frames, "right_calf")
    left_arm, right_arm = curve(frames, "left_arm"), curve(frames, "right_arm")
    samples = []
    for i in range(len(frames)):
        left_swing = left_thigh[i] >= right_thigh[i]
        calf = left_calf[i] if left_swing else right_calf[i]
        arm = right_arm[i] if left_swing else left_arm[i]
        lift = max(0.0, calf)
        samples.append({"left_swing": left_swing, "stride_rad": round(0.022 + 0.058 * lift, 6),
                        "knee_lift_rad": round(0.085 + 0.185 * lift, 6),
                        "ankle_lift_rad": round(-0.024 - 0.052 * lift, 6),
                        "arm_drive_rad": round(0.10 + 0.22 * max(0.0, arm), 6)})
    payload = {"format": "cao_h1_motion_reference_v1", "source_motion": source["source"],
               "samples": samples, "note": "Human motion retargeted to bounded H1 offsets; physics remains active."}
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {args.output} with {len(samples)} retargeted samples")


if __name__ == "__main__": main()
