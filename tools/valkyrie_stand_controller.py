"""Safe standing controller scaffold for the isolated Valkyrie candidate.

The robot posture is controlled exclusively through its imported torque motors.
The optional base fixture is a bounded virtual safety gantry used while
calibrating contacts and collecting demonstrations; it must be disabled before
calling any result free-standing or integrating the model into the live app.
"""
import math

import mujoco
import numpy as np


class ValkyrieStandController:
    """Torque PD posture control plus an opt-in, bounded calibration fixture."""

    def __init__(self, model: mujoco.MjModel, safety_fixture: bool = False):
        self.model = model
        self.safety_fixture = safety_fixture
        self.joint_ids = model.actuator_trnid[:, 0]
        self.qpos_addresses = np.array([model.jnt_qposadr[joint] for joint in self.joint_ids])
        self.dof_addresses = np.array([model.jnt_dofadr[joint] for joint in self.joint_ids])
        ranges = model.jnt_range[self.joint_ids]
        self.targets = np.clip(np.zeros(model.nu), ranges[:, 0] + 0.02, ranges[:, 1] - 0.02)
        self.kp = np.full(model.nu, 2.0)
        self.kd = np.full(model.nu, 0.5)
        for actuator in range(model.nu):
            name = model.actuator(actuator).name
            if "Hip" in name or "Knee" in name:
                self.kp[actuator], self.kd[actuator] = 120.0, 12.0
            elif "Ankle" in name:
                self.kp[actuator], self.kd[actuator] = 70.0, 8.0
            elif "torso" in name:
                self.kp[actuator], self.kd[actuator] = 80.0, 8.0
            elif "Shoulder" in name:
                self.kp[actuator], self.kd[actuator] = 15.0, 2.0
            elif "Elbow" in name:
                self.kp[actuator], self.kd[actuator] = 10.0, 1.0
        pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.mass = float(model.body_subtreemass[pelvis])

    @staticmethod
    def _roll_pitch(quaternion: np.ndarray) -> tuple[float, float]:
        w, x, y, z = quaternion
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.atan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch

    def step(self, data: mujoco.MjData) -> None:
        """Apply torque-limited posture control and the optional safety fixture."""
        torque = self.kp * (self.targets - data.qpos[self.qpos_addresses])
        torque -= self.kd * data.qvel[self.dof_addresses]
        data.ctrl[:] = np.clip(
            torque, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1]
        )
        data.qfrc_applied[:] = 0.0
        if not self.safety_fixture:
            return

        roll, pitch = self._roll_pitch(data.qpos[3:7])
        horizontal_stiffness = 5000.0
        horizontal_damping = 2.0 * math.sqrt(horizontal_stiffness * self.mass)
        # This fixture cannot lift the robot: vertical support remains the two
        # foot contacts. It only prevents runaway horizontal drift and tip-over
        # while the real contact-aware balance policy is trained.
        fixture = np.array(
            [
                -horizontal_stiffness * data.qpos[0] - horizontal_damping * data.qvel[0],
                -horizontal_stiffness * data.qpos[1] - horizontal_damping * data.qvel[1],
                0.0,
                -2000.0 * roll - 150.0 * data.qvel[3],
                -2000.0 * pitch - 150.0 * data.qvel[4],
                -100.0 * data.qvel[5],
            ]
        )
        data.qfrc_applied[:6] = np.clip(fixture, -5000.0, 5000.0)

    def foot_normal_forces(self, data: mujoco.MjData) -> tuple[float, float]:
        """Return physical normal forces from the two sole collision boxes."""
        left_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "leftFoot")
        right_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "rightFoot")
        forces = [0.0, 0.0]
        wrench = np.zeros(6)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            first_body = self.model.geom_bodyid[contact.geom1]
            second_body = self.model.geom_bodyid[contact.geom2]
            mujoco.mj_contactForce(self.model, data, contact_index, wrench)
            if left_body in (first_body, second_body):
                forces[0] += max(0.0, float(wrench[0]))
            if right_body in (first_body, second_body):
                forces[1] += max(0.0, float(wrench[0]))
        return tuple(forces)
