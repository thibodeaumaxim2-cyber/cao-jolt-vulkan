#!/usr/bin/env python3
"""Kinematic regression for alternating Valkyrie IK gait targets."""
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from valkyrie_ik import ValkyrieGaitSolver


def main():
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets" / "valkyrie" / "valkyrie.xml"))
    gait = ValkyrieGaitSolver(model)
    left_start, left_swing, right_swing = gait.target(.0), gait.target(.30), gait.target(.90)
    if left_start[1] != "left" or right_swing[1] != "right":
        raise SystemExit("gait solver did not alternate swing legs")
    left_lift = left_swing[2]["left"][2] - left_start[2]["left"][2]
    if left_lift < .015:
        raise SystemExit(f"left swing lift too small: {left_lift:.4f} m")
    values = np.asarray(list(left_swing[0].values()) + list(right_swing[0].values()))
    if not np.isfinite(values).all():
        raise SystemExit("gait solver returned nonfinite motor targets")
    print(f"valkyrie_ik_gait_targets={len(left_swing[0])} left_swing_lift_m={left_lift:.3f}")


if __name__ == "__main__":
    main()
