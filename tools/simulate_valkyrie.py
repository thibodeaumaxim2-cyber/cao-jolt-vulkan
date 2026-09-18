#!/usr/bin/env python3
"""Run a deterministic Valkyrie standing simulation and report measured state."""
import argparse
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from valkyrie_stand_controller import ValkyrieStandController


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("assisted", "free"), default="assisted")
    parser.add_argument("--steps", type=int, default=2500)
    parser.add_argument("--impulse-n", type=float, default=300.0)
    args = parser.parse_args()
    if args.steps <= 0:
        raise SystemExit("--steps must be positive")

    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/valkyrie/valkyrie.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    controller = ValkyrieStandController(model, safety_fixture=args.mode == "assisted")
    orientation_sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "pelvis_orientation")
    lowest_height = float(data.qpos[2])
    maximum_drift = 0.0
    maximum_tilt = 0.0
    left_peak_force = right_peak_force = 0.0
    fallen = False
    for step in range(args.steps):
        controller.step(data)
        if args.mode == "assisted" and args.steps // 10 <= step < args.steps // 10 + 50:
            data.qfrc_applied[0] += args.impulse_n
        mujoco.mj_step(model, data)
        left_force, right_force = controller.foot_normal_forces(data)
        left_peak_force = max(left_peak_force, left_force)
        right_peak_force = max(right_peak_force, right_force)
        lowest_height = min(lowest_height, float(data.qpos[2]))
        maximum_drift = max(maximum_drift, float(np.linalg.norm(data.qpos[:2])))
        orientation_start = model.sensor_adr[orientation_sensor]
        orientation = data.sensordata[orientation_start:orientation_start + 4]
        maximum_tilt = max(maximum_tilt, 2.0 * float(np.arccos(np.clip(abs(orientation[0]), 0.0, 1.0))))
        fallen = fallen or data.qpos[2] < 0.7

    print(
        f"mode={args.mode} duration_s={args.steps * model.opt.timestep:.2f} "
        f"fallen={fallen} min_pelvis_height_m={lowest_height:.3f} "
        f"max_drift_m={maximum_drift:.3f} max_tilt_rad={maximum_tilt:.3f} "
        f"peak_left_contact_n={left_peak_force:.1f} peak_right_contact_n={right_peak_force:.1f}"
    )
    if args.mode == "assisted" and (fallen or not np.isfinite(data.qpos).all()):
        raise SystemExit("assisted Valkyrie simulation failed")


if __name__ == "__main__":
    main()
