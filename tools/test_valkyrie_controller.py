#!/usr/bin/env python3
"""Regression for the isolated Valkyrie MuJoCo articulation and PD controller.

This is intentionally an actuator-mapping test in zero gravity.  It proves
that all 58 imported robot joints accept bounded torque commands and recover
from safe in-range perturbations.  It is not a free-standing balance claim;
that controller remains a separate follow-up before any live integration.
"""
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "assets/valkyrie/valkyrie.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    if (model.nq, model.nv, model.nu, model.njnt) != (66, 65, 58, 60):
        raise SystemExit("unexpected Valkyrie MuJoCo articulation dimensions")
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    joint_ids = model.actuator_trnid[:, 0]
    qpos_addresses = np.array([model.jnt_qposadr[joint] for joint in joint_ids])
    dof_addresses = np.array([model.jnt_dofadr[joint] for joint in joint_ids])
    ranges = model.jnt_range[joint_ids]
    # Zero is valid for most joints; one-sided joints get a small safe offset.
    targets = np.clip(np.zeros(model.nu), ranges[:, 0] + 0.02, ranges[:, 1] - 0.02)
    perturbation = np.where(np.arange(model.nu) % 2, 0.01, -0.01)
    data.qpos[qpos_addresses] = np.clip(
        targets + perturbation, ranges[:, 0] + 0.001, ranges[:, 1] - 0.001
    )
    mujoco.mj_forward(model, data)
    initial_error = float(np.max(np.abs(data.qpos[qpos_addresses] - targets)))

    for _ in range(2400):
        torque = targets - data.qpos[qpos_addresses] - 0.2 * data.qvel[dof_addresses]
        data.ctrl[:] = np.clip(
            torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
        )
        mujoco.mj_step(model, data)

    final_error = float(np.max(np.abs(data.qpos[qpos_addresses] - targets)))
    if not np.isfinite(data.qpos).all() or final_error >= initial_error * 0.8:
        raise SystemExit("Valkyrie actuator PD recovery regression failed")
    print(
        f"valkyrie_actuators={model.nu} initial_error_rad={initial_error:.5f} "
        f"final_error_rad={final_error:.5f}"
    )


if __name__ == "__main__":
    main()
