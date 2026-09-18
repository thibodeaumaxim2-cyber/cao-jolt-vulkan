"""Damped-least-squares IK for the isolated Valkyrie MuJoCo model."""
from __future__ import annotations

import mujoco
import numpy as np


def solve_site_position(model, data, site_name: str, target, *, iterations=40,
                        damping=1e-2, step_size=0.7, max_delta=0.08,
                        joint_names=()) -> float:
    """Move a free-joint model site toward target and return final error."""
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site < 0:
        raise ValueError(f"unknown site: {site_name}")
    target = np.asarray(target, dtype=float)
    dofs = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in joint_names]
    if not dofs:
        dofs = list(range(6, model.nv))  # leave the floating pelvis untouched
    jac = np.zeros((3, model.nv))
    for _ in range(iterations):
        mujoco.mj_forward(model, data)
        error = target - data.site_xpos[site]
        if np.linalg.norm(error) < 1e-3:
            break
        mujoco.mj_jacSite(model, data, jac, None, site)
        j = jac[:, dofs]
        dq = j.T @ np.linalg.solve(j @ j.T + damping * np.eye(3), error)
        dq = np.clip(step_size * dq, -max_delta, max_delta)
        for dof, value in zip(dofs, dq):
            joint = model.dof_jntid[dof]
            qpos = model.jnt_qposadr[joint]
            data.qpos[qpos] += value
            if model.jnt_limited[joint]:
                data.qpos[qpos] = np.clip(data.qpos[qpos], model.jnt_range[joint, 0], model.jnt_range[joint, 1])
    mujoco.mj_forward(model, data)
    return float(np.linalg.norm(target - data.site_xpos[site]))


def solve_standing_feet(model, data, pelvis_height=1.18) -> float:
    """Solve both sole positions while preserving the imported upright pose."""
    data.qpos[2] = pelvis_height
    left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_sole")
    right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_sole")
    if left < 0 or right < 0:
        raise ValueError("Valkyrie sole sites are missing")
    targets = {"left_sole": data.site_xpos[left].copy(), "right_sole": data.site_xpos[right].copy()}
    error = 0.0
    leg_names = ("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
                 "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll")
    for name, target in targets.items():
        error = max(error, solve_site_position(model, data, name, target, joint_names=leg_names))
    return error
