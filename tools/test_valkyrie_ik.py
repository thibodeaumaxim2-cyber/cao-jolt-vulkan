#!/usr/bin/env python3
"""Regression for Valkyrie's MuJoCo Jacobian IK."""
from pathlib import Path
import mujoco
import numpy as np
from valkyrie_ik import solve_site_position

ROOT = Path(__file__).resolve().parents[1]
model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/valkyrie/valkyrie.xml"))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0) if model.nkey else mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_sole")
target = data.site_xpos[site].copy() + np.array([0.05, 0.0, 0.02])
error = solve_site_position(model, data, "right_sole", target, joint_names=(
    "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"))
if error > 0.004:
    raise SystemExit(f"Valkyrie IK did not converge: {error:.5f} m")
print(f"Valkyrie IK pass: final_error_m={error:.5f}")
