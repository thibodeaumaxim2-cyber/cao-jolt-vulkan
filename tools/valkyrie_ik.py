"""Damped-least-squares IK for the isolated Valkyrie MuJoCo model."""
from __future__ import annotations

import mujoco
import numpy as np


LEG_JOINTS = {
    "left": ("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"),
    "right": ("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"),
}


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


class ValkyrieGaitSolver:
    """Alternating-support sole trajectory generator backed by DLS IK.

    It produces bounded motor position targets only.  Dynamic balance and
    contact switching remain the responsibility of a separate controller.
    """

    def __init__(self, model, pelvis_height=1.16, cycle_seconds=1.2,
                 step_length=0.10, swing_height=0.045):
        self.model = model
        self.pelvis_height = pelvis_height
        self.cycle_seconds = cycle_seconds
        self.step_length = step_length
        self.swing_height = swing_height
        self.reference = mujoco.MjData(model)
        self.reference.qpos[2] = pelvis_height
        mujoco.mj_forward(model, self.reference)
        self.sole_sites = {
            "left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_sole"),
            "right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_sole"),
        }
        if min(self.sole_sites.values()) < 0:
            raise ValueError("Valkyrie sole sites are missing")
        self.nominal = {side: self.reference.site_xpos[site].copy() for side, site in self.sole_sites.items()}
        self.actuator_targets = {}
        for side, names in LEG_JOINTS.items():
            for name in names:
                joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self.actuator_targets[name] = (joint, model.jnt_qposadr[joint])

    def target(self, time_s):
        """Return {motor-name: radian target}, swing leg, and sole targets."""
        phase = (time_s % self.cycle_seconds) / self.cycle_seconds
        swing_side = "left" if phase < .5 else "right"
        swing_phase = phase * 2.0 if swing_side == "left" else (phase - .5) * 2.0
        data = mujoco.MjData(self.model)
        data.qpos[:] = self.reference.qpos
        swing_target = self.nominal[swing_side].copy()
        # Smooth zero-velocity lift and placement. MuJoCo uses Z-up.
        swing_target[0] += self.step_length * (swing_phase - .5)
        swing_target[2] += self.swing_height * np.sin(np.pi * swing_phase)
        solve_site_position(self.model, data, f"{swing_side}_sole", swing_target,
                            iterations=56, damping=2e-2, step_size=.55, max_delta=.05,
                            joint_names=LEG_JOINTS[swing_side])
        mujoco.mj_forward(self.model, data)
        targets = {}
        for name, (joint, qpos) in self.actuator_targets.items():
            value = data.qpos[qpos]
            if self.model.jnt_limited[joint]:
                value = np.clip(value, self.model.jnt_range[joint, 0] + .01, self.model.jnt_range[joint, 1] - .01)
            targets[name] = float(value)
        soles = {side: data.site_xpos[site].copy() for side, site in self.sole_sites.items()}
        return targets, swing_side, soles
