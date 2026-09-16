#!/usr/bin/env python3
"""Train and export the compact H1 gait policy consumed by MuJoCoBridge.

This is supervised motion imitation from the bundled Mesh2Motion CC0 walk.
The exported linear policy is deliberately tiny so runtime inference remains in
C++ and Python/PyTorch are never needed by the application.
"""
import argparse
import json
import math
from pathlib import Path

import torch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("assets/h1_locomotion_policy.json"))
    parser.add_argument("--motion", type=Path, default=Path("assets/motion/mesh2motion_walk.json"),
                        help="Blender-extracted CC0 timing curves")
    parser.add_argument("--epochs", type=int, default=1200)
    args = parser.parse_args()
    torch.manual_seed(7)
    if not args.motion.exists():
        raise SystemExit(f"Missing {args.motion}. Run tools/extract_mesh2motion_walk.py with Blender first.")
    motion = json.loads(args.motion.read_text())
    frames = motion.get("frames", [])
    if len(frames) < 16:
        raise SystemExit("Motion export has too few frames")

    def dominant_curve(name: str):
        values = [frame["rotations_xyz_rad"][name] for frame in frames]
        axes = list(zip(*values))
        curve = max(axes, key=lambda axis: max(axis) - min(axis))
        mean = sum(curve) / len(curve)
        scale = max(1.0e-5, max(abs(value - mean) for value in curve))
        return [(value - mean) / scale for value in curve]

    left_thigh, right_thigh = dominant_curve("left_thigh"), dominant_curve("right_thigh")
    left_calf, right_calf = dominant_curve("left_calf"), dominant_curve("right_calf")
    samples = len(frames)
    phase = torch.tensor([2.0 * math.pi * index / samples for index in range(samples)], dtype=torch.float32)
    # Human thigh opposition determines which H1 leg is allowed to swing. Calf
    # flexion determines lift magnitude; this keeps the source motion's timing
    # while H1 limits remain deliberately conservative.
    side_values = [1.0 if left_thigh[index] >= right_thigh[index] else -1.0 for index in range(samples)]
    knee_values = [max(0.0, (left_calf[index] if side_values[index] > 0 else right_calf[index])) for index in range(samples)]
    knee_peak = max(1.0e-5, max(knee_values))
    lift_values = [value / knee_peak for value in knee_values]
    side = torch.tensor(side_values, dtype=torch.float32)
    lift = torch.tensor(lift_values, dtype=torch.float32)
    # Inputs: source phase, equilibrium score, and retargeted swing-side sign.
    x = torch.stack((torch.sin(phase), torch.cos(phase), torch.ones_like(phase), side), dim=1)
    # Teacher output: H1-safe amplitudes driven by CC0 human walk timing.
    hip = 0.024 + 0.060 * lift
    y = torch.stack((hip, 0.090 + 0.190 * lift, -0.025 - 0.055 * lift), dim=1)
    model = torch.nn.Linear(4, 3)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.03)
    for _ in range(args.epochs):
        loss = torch.mean((model(x) - y) ** 2)
        optimizer.zero_grad(); loss.backward(); optimizer.step()
    with torch.no_grad():
        mse = float(torch.mean((model(x) - y) ** 2))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = {"format": "cao_h1_linear_policy_v1", "source_motion": motion.get("source", "unknown"),
               "motion_samples": samples, "inputs": ["sin_phase", "cos_phase", "equilibrium", "swing_side"],
               "outputs": ["hip_sweep_rad", "knee_lift_rad", "ankle_lift_rad"],
               "weight": model.weight.detach().tolist(), "bias": model.bias.detach().tolist(), "training_mse": mse}
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {args.output} (mse={mse:.8f})")


if __name__ == "__main__":
    main()
