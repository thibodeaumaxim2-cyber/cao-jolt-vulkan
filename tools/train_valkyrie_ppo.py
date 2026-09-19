#!/usr/bin/env python3
"""Train a grounded, motor-only Valkyrie stance/walk policy with PPO.

The resulting checkpoint is deliberately offline-only.  It must pass
evaluate_valkyrie_ppo.py before any native Vulkan integration is considered.
"""
import argparse
import json
from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

try:
    from valkyrie_ik import ValkyrieGaitSolver
except ModuleNotFoundError:  # supports import by the evaluation harness
    from tools.valkyrie_ik import ValkyrieGaitSolver

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "assets" / "valkyrie" / "valkyrie.xml"
STANCE = ROOT / "assets" / "valkyrie" / "stance_targets.json"
LEGS = (
    "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
    "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
)


class ValkyrieEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, walking: bool = False):
        self.m = mujoco.MjModel.from_xml_path(str(XML))
        self.d = mujoco.MjData(self.m)
        self.walking = walking
        self.joints = np.asarray([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name) for name in LEGS])
        self.actuators = np.asarray([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in LEGS])
        # Python exposes actuator_trnid as [actuator, transmission-slot],
        # unlike the flat C array used by the native bridge.
        source_joints = self.m.actuator_trnid[self.actuators, 0]
        self.qadr = self.m.jnt_qposadr[source_joints]
        self.dadr = self.m.jnt_dofadr[source_joints]
        self.home = np.zeros(len(LEGS), dtype=np.float32)
        if STANCE.exists():
            posture = json.loads(STANCE.read_text())
            values = np.asarray(posture["symmetric_targets_rad"], dtype=np.float32)
            if posture.get("format") != "cao_valkyrie_stance_targets_v1" or values.shape != (6,):
                raise RuntimeError("Invalid Valkyrie stance target file")
            self.home = np.tile(values, 2)
        self.kp = np.asarray([90, 110, 180, 220, 120, 95] * 2, dtype=np.float32)
        self.kd = np.asarray([8, 10, 18, 20, 12, 10] * 2, dtype=np.float32)
        # Standing begins as a small residual around the motor-held contact
        # pose. Walking expands this only after a checkpoint has passed.
        self.action_scale = np.asarray([.10, .08, .16, .20, .12, .07] * 2 if walking else
                                       [.025, .020, .040, .050, .030, .020] * 2, dtype=np.float32)
        self.stand_height = 1.16
        self.min_height = 1.08
        self.left_foot = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "leftFoot")
        self.right_foot = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "rightFoot")
        self.action_space = spaces.Box(-1, 1, shape=(len(LEGS),), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(34,), dtype=np.float32)
        self.step_count = 0
        self.gait_time = 0.0
        self.gait = ValkyrieGaitSolver(self.m) if walking else None

    def _contacts(self):
        values = np.zeros(2, dtype=np.float32)
        for index in range(self.d.ncon):
            contact = self.d.contact[index]
            bodies = self.m.geom_bodyid[contact.geom]
            if self.left_foot in bodies:
                values[0] = 1.0
            if self.right_foot in bodies:
                values[1] = 1.0
        return values

    def _observation(self):
        w, x, y, z = self.d.qpos[3:7]
        gravity = np.asarray((2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)))
        phase = 2 * np.pi * (self.step_count * self.m.opt.timestep / .8)
        return np.r_[self.d.qvel[3:6] * .25, gravity, self.d.qpos[self.qadr] - self.home,
                     self.d.qvel[self.dadr] * .05, self._contacts(), np.sin(phase), np.cos(phase)].astype(np.float32)

    def _motor_balance_target(self):
        """IMU-only ankle/hip correction; never applies a floating-base force."""
        w, x, y, z = self.d.qpos[3:7]
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arctan2(2 * (w * y - z * x), 1 - 2 * (y * y + z * z))
        roll_rate, pitch_rate = self.d.qvel[3], self.d.qvel[4]
        correction = np.zeros(len(LEGS), dtype=np.float32)
        # Leg ordering: yaw, roll, pitch, knee, ankle pitch, ankle roll.
        # Both ankles counter body tilt; the hip terms share the correction
        # to keep the torso over the double-support polygon.
        for base in (0, 6):
            correction[base + 1] = np.clip(-.10 * roll - .025 * roll_rate, -.08, .08)
            correction[base + 2] = np.clip(-.08 * pitch - .020 * pitch_rate, -.10, .10)
            correction[base + 4] = np.clip(-.28 * pitch - .045 * pitch_rate, -.16, .16)
            correction[base + 5] = np.clip(-.30 * roll - .050 * roll_rate, -.14, .14)
        return correction

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.m, self.d)
        # At z=1.16 the two sole boxes make eight physical ground contacts.
        # Starting at the old 1.18 m left both soles airborne and forced PPO
        # to learn from a falling state instead of a double-support stance.
        self.d.qpos[2] = self.stand_height
        self.d.qpos[self.qadr] = self.home + self.np_random.normal(0, .002, len(LEGS))
        self.d.qvel[:] = self.np_random.normal(0, .002, self.m.nv)
        self.step_count = 0
        self.gait_time = 0.0
        mujoco.mj_forward(self.m, self.d)
        if self._contacts().sum() < 2:
            raise RuntimeError("Valkyrie reset did not establish double-support foot contacts")
        return self._observation(), {}

    def step(self, action):
        before_x = float(self.d.qpos[0])
        contacts_before = self._contacts()
        target = self.home.copy()
        swing_side = None
        if self.gait:
            gait_targets, swing_side, _ = self.gait.target(self.gait_time)
            target = np.asarray([gait_targets[name] for name in LEGS], dtype=np.float32)
        target += self._motor_balance_target() + np.clip(action, -1, 1) * self.action_scale
        for _ in range(10):  # 50 Hz policy, native MuJoCo physics substeps
            torque = self.kp * (target - self.d.qpos[self.qadr]) - self.kd * self.d.qvel[self.dadr]
            self.d.ctrl[self.actuators] = np.clip(torque, self.m.actuator_ctrlrange[self.actuators, 0], self.m.actuator_ctrlrange[self.actuators, 1])
            self.d.qfrc_applied[:] = 0.0  # policy may use motors only
            mujoco.mj_step(self.m, self.d)
        self.step_count += 10
        height = float(self.d.qpos[2])
        upright = float(self.d.xmat[1, 8])
        contacts = self._contacts()
        height_error = abs(height - self.stand_height)
        vertical_speed = abs(float(self.d.qvel[2]))
        angular_speed = float(np.linalg.norm(self.d.qvel[3:6]))
        double_support = float(contacts.sum() == 2)
        # The swing phase is allowed to advance only while the opposite foot
        # provides measured support and the IMU remains upright. This prevents
        # an open-loop IK trajectory from stepping into a fall.
        if swing_side:
            support_index = 1 if swing_side == "left" else 0
            if contacts_before[support_index] and upright >= .70:
                self.gait_time += .02
        fallen = height < self.min_height or upright < .70 or not np.isfinite(self.d.qpos).all()
        progress = float(self.d.qpos[0]) - before_x
        effort = float(np.mean(np.square(self.d.ctrl[self.actuators])))
        reward = 3.0 * np.exp(-24.0 * height_error) + 2.0 * max(0.0, upright) + 1.5 * double_support
        if self.walking:
            reward += 4.0 * progress
        reward -= .35 * vertical_speed + .08 * angular_speed + .00003 * effort + .02 * float(np.mean(np.square(action))) + (15.0 if fallen else 0.0)
        return self._observation(), reward, fallen, False, {
            "height": height,
            "upright": upright,
            "progress": progress,
            "contacts": contacts.tolist(),
            "double_support": bool(double_support),
            "height_failure": height < self.min_height,
            "tilt_failure": upright < .70,
            "swing_leg": swing_side,
        }


def make_env(walking: bool, seed: int):
    """Create a picklable worker factory for parallel MuJoCo rollouts."""
    def factory():
        environment = ValkyrieEnv(walking=walking)
        environment.reset(seed=seed)
        return environment
    return factory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=1_000_000)
    parser.add_argument("--walking", action="store_true")
    parser.add_argument("--workers", type=int, default=8,
                        help="independent MuJoCo rollout processes (1 disables multiprocessing)")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "valkyrie_ppo")
    parser.add_argument("--init-model", type=Path,
                        help="passing standing checkpoint used to initialize walking training")
    args = parser.parse_args()
    if args.workers < 1:
        raise SystemExit("--workers must be at least 1")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    factories = [make_env(args.walking, seed) for seed in range(args.workers)]
    # Each process owns its own MuJoCo model/data pair.  This avoids shared
    # simulation state while collecting PPO rollouts across CPU cores.
    env = DummyVecEnv(factories) if args.workers == 1 else SubprocVecEnv(factories, start_method="spawn")
    policy = (PPO.load(args.init_model, env=env, device="auto") if args.init_model else
              PPO("MlpPolicy", env, verbose=1, n_steps=512, batch_size=256, learning_rate=3e-4,
                  policy_kwargs={"net_arch": [256, 256]}))
    policy.learn(total_timesteps=args.steps)
    policy.save(args.output)
    env.close()
    print(f"saved {args.output}.zip")


if __name__ == "__main__":
    main()
