"""Full-body joint feedback with a double-support wrench estimate.

The estimated wrench is used only to calculate motor feedforward. MuJoCo
computes actual ground reaction forces; nothing is applied to the free base.
"""
import mujoco
import numpy as np


class ValkyrieBalance:
    def __init__(self, model):
        self.m = model
        # Explicit motor damping at these gains requires a 1 kHz inner loop.
        model.opt.timestep = .001
        joints = model.actuator_trnid[:, 0]
        self.qa = model.jnt_qposadr[joints]
        self.va = model.jnt_dofadr[joints]
        self.home = np.zeros(model.nu)
        self.kp = np.full(model.nu, 80.)
        self.kd = np.full(model.nu, 8.)
        for i in range(model.nu):
            name = model.actuator(i).name
            if 'Hip' in name or 'Knee' in name:
                self.kp[i], self.kd[i] = 800., 50.
            if 'Ankle' in name:
                self.kp[i], self.kd[i] = 600., 35.
            if 'torso' in name:
                self.kp[i], self.kd[i] = 500., 35.
            if any(part in name for part in ('Finger', 'Thumb', 'Pinky')):
                self.kp[i], self.kd[i] = 5., .5
            if 'HipPitch' in name or 'AnklePitch' in name:
                self.home[i] = -.2
            if 'KneePitch' in name:
                self.home[i] = .4
        self.home = np.clip(self.home, model.jnt_range[joints, 0], model.jnt_range[joints, 1])
        self.feet = [model.site(name).id for name in ('left_sole', 'right_sole')]
        self.root = int(model.joint('valkyrie_root').qposadr[0])
        self.root_dof = int(model.joint('valkyrie_root').dofadr[0])
        self.J = np.zeros((12, model.nv))

    def reset(self, data):
        data.qpos[self.qa] = self.home
        mujoco.mj_forward(self.m, data)
        # Reset only: place the sole surfaces on the floor for this pose.
        data.qpos[self.root + 2] -= min(data.site_xpos[self.feet, 2])
        mujoco.mj_forward(self.m, data)

    def control(self, data, target=None):
        mujoco.mj_forward(self.m, data)
        for i, foot in enumerate(self.feet):
            mujoco.mj_jacSite(self.m, data, self.J[6*i:6*i+3], self.J[6*i+3:6*i+6], foot)
        base = slice(self.root_dof, self.root_dof + 6)
        A = self.J[:, base].T
        # Minimum-norm support wrench satisfying the free-base bias equation.
        wrench = A.T @ np.linalg.solve(A @ A.T, data.qfrc_bias[base])
        feedforward = data.qfrc_bias - self.J.T @ wrench
        target = self.home if target is None else target
        torque = self.kp*(target-data.qpos[self.qa]) - self.kd*data.qvel[self.va] + feedforward[self.va]
        data.ctrl[:] = np.clip(torque, self.m.actuator_ctrlrange[:, 0], self.m.actuator_ctrlrange[:, 1])
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0
