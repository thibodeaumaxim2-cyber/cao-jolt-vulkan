"""Motor-only MuJoCo environment for learning a Valkyrie standing policy."""

from __future__ import annotations

from pathlib import Path
from .valkyrie_balance import ValkyrieBalance

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np


class ValkyrieStandEnv(gym.Env):
    """NASA Valkyrie standing stabilization environment.

    The policy runs at 50 Hz while the PD controller runs at every MuJoCo
    simulation step.  All support forces must come from foot contacts and leg
    motors: this environment never applies a floating-base force or pose edit.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    leg_joint_names = (
        "leftHipYaw", "leftHipRoll", "leftHipPitch",
        "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
        "rightHipYaw", "rightHipRoll", "rightHipPitch",
        "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
    )

    def __init__(self, model_path: str | Path, render_mode: str | None = None):
        super().__init__()
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"MJCF model path not found: {model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self.balance = ValkyrieBalance(self.model)

        self.dt_sim = self.model.opt.timestep
        self.dt_policy = 0.02
        self.n_substeps = max(1, int(round(self.dt_policy / self.dt_sim)))

        joints = self._resolve_ids(mujoco.mjtObj.mjOBJ_JOINT, self.leg_joint_names)
        self.leg_qpos_indices = self.model.jnt_qposadr[joints].astype(np.int32)
        self.leg_dof_indices = self.model.jnt_dofadr[joints].astype(np.int32)
        self.leg_actuator_indices = self._resolve_ids(mujoco.mjtObj.mjOBJ_ACTUATOR, self.leg_joint_names)

        root_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "valkyrie_root")
        if root_joint == -1 or self.model.jnt_type[root_joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("MJCF must contain the free joint 'valkyrie_root'.")
        self.root_qposadr = int(self.model.jnt_qposadr[root_joint])
        self.root_dofadr = int(self.model.jnt_dofadr[root_joint])

        # Bent knees avoid the kinematic singularity of a fully straight leg.
        self.q0_legs = np.array(
            [0.0, 0.0, -0.2, 0.4, -0.2, 0.0] * 2, dtype=np.float64
        )
        self.ctrl_low = self.model.actuator_ctrlrange[self.leg_actuator_indices, 0]
        self.ctrl_high = self.model.actuator_ctrlrange[self.leg_actuator_indices, 1]

        self.target_pelvis_height = 1.1526838687581835
        self.min_height = self.target_pelvis_height - .05
        self.action_scale = 0.025
        self.m, self.d = self.model, self.data  # Existing training/evaluation interface.
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(12,), dtype=np.float32)

        # pelvis height (1), projected gravity (3), root velocity (6), leg
        # position error (12), leg velocity (12), feet forces (2), action (12).
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(48,), dtype=np.float32)
        self.prev_action = np.zeros(12, dtype=np.float32)
        self.lf_body_id, self.rf_body_id = self._resolve_ids(
            mujoco.mjtObj.mjOBJ_BODY, ("leftFoot", "rightFoot")
        )
        self.total_weight = float(self.model.body_mass.sum() * abs(self.model.opt.gravity[2]))

    def _resolve_ids(self, object_type: mujoco.mjtObj, names: tuple[str, ...]) -> np.ndarray:
        ids = np.asarray([mujoco.mj_name2id(self.model, object_type, name) for name in names], dtype=np.int32)
        missing = [name for name, object_id in zip(names, ids) if object_id == -1]
        if missing:
            raise ValueError(f"MJCF objects not found: {', '.join(missing)}")
        return ids

    def _get_contact_forces(self) -> tuple[float, float]:
        """Return normal contact-force magnitudes for the two feet."""
        forces = np.zeros(2, dtype=np.float64)
        contact_force = np.zeros(6, dtype=np.float64)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            bodies = self.model.geom_bodyid[[contact.geom1, contact.geom2]]
            mujoco.mj_contactForce(self.model, self.data, index, contact_force)
            if self.lf_body_id in bodies:
                forces[0] += contact_force[0]
            if self.rf_body_id in bodies:
                forces[1] += contact_force[0]
        return float(forces[0]), float(forces[1])

    def _root_pose(self) -> tuple[np.ndarray, np.ndarray]:
        start = self.root_qposadr
        return self.data.qpos[start : start + 3], self.data.qpos[start + 3 : start + 7]

    def _get_obs(self) -> np.ndarray:
        root_pos, pelvis_quat = self._root_pose()
        rotation = np.empty(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rotation, pelvis_quat)
        rotation = rotation.reshape(3, 3)
        projected_gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        root_velocity = self.data.qvel[self.root_dofadr : self.root_dofadr + 6]
        root_velocity_body = rotation.T @ root_velocity[:3]
        root_angular_velocity_body = root_velocity[3:]  # Free-joint angular velocity is already local.
        left_force, right_force = self._get_contact_forces()
        force_scale = max(self.total_weight, 1.0)
        return np.concatenate((
            [root_pos[2] - self.target_pelvis_height],
            projected_gravity,
            root_velocity_body,
            root_angular_velocity_body,
            self.data.qpos[self.leg_qpos_indices] - self.q0_legs,
            self.data.qvel[self.leg_dof_indices] * 0.1,
            [left_force / force_scale, right_force / force_scale],
            self.prev_action,
        )).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        root_pos, root_quat = self._root_pose()
        root_pos[:] = (0.0, 0.0, 1.18)
        root_quat[:] = (1.0, 0.0, 0.0, 0.0)
        self.balance.reset(self.data)
        self.target_pelvis_height = float(root_pos[2])
        self.min_height = self.target_pelvis_height - .05
        self.data.qpos[self.leg_qpos_indices] += self.np_random.uniform(-0.002, 0.002, 12)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.prev_action.fill(0.0)
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action_clipped = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        if action_clipped.shape != self.action_space.shape:
            raise ValueError(f"Expected action shape {self.action_space.shape}, got {action_clipped.shape}")
        previous_action = self.prev_action.copy()
        target = self.balance.home.copy()
        target[self.leg_actuator_indices] += action_clipped * self.action_scale
        for _ in range(self.n_substeps):
            self.balance.control(self.data, target)
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        root_pos, pelvis_quat = self._root_pose()
        upright = float(1.0 - 2.0 * (pelvis_quat[1] ** 2 + pelvis_quat[2] ** 2))
        left_force, right_force = self._get_contact_forces()
        torques = self.data.ctrl[self.leg_actuator_indices]
        height_reward = np.exp(-25.0 * (root_pos[2] - self.target_pelvis_height) ** 2)
        support_reward = 1.0 if left_force > 100.0 and right_force > 100.0 else 0.2
        torque_penalty = 0.01 * np.square(torques / np.maximum(np.abs(self.ctrl_low), self.ctrl_high)).mean()
        action_rate_penalty = 0.05 * np.square(action_clipped - previous_action).sum()
        reward = 0.45 * height_reward + 0.35 * max(upright, 0.0) + 0.20 * support_reward
        reward -= torque_penalty + action_rate_penalty
        self.prev_action = action_clipped.copy()

        terminated = bool(root_pos[2] < self.min_height or upright < 0.98 or not np.isfinite(self.data.qpos).all())
        return self._get_obs(), float(reward), terminated, False, {
            "pelvis_height": float(root_pos[2]),
            "upright": upright,
            "left_foot_force": left_force,
            "right_foot_force": right_force,
            "knee_pitch_torque": float(torques[3]),
            "double_support": bool(left_force > 100 and right_force > 100),
            "height_failure": bool(root_pos[2] < self.min_height),
            "tilt_failure": bool(upright < .98),
        }
