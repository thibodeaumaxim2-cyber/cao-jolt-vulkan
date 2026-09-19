"""Slow, level-ground stepping through joint torques and measured contacts."""
import mujoco
import numpy as np


class ValkyrieWalk:
    TRANSFER = 2.0
    SWING = 1.4

    def __init__(self, model, data, balance):
        self.m, self.b = model, balance
        mujoco.mj_forward(model, data)
        self.feet = data.site_xpos[balance.feet].copy()
        self.feet[:, 2] = 0
        self.fs = self.feet.copy()
        self.p = data.qpos[:3].copy()
        self.height = self.p[2] - .035
        self.begin = self.p.copy()
        self.end = self.p.copy()
        self.end[2] = self.height
        self.weights = np.ones(12)
        self.initial_weights = self.weights.copy()
        self.phase, self.elapsed, self.side = 'crouch', 0., 0
        self.active_swing = -1
        self.old_jac = None
        self.J = np.zeros((12, model.nv))
        self.va = []
        for side in ('left', 'right'):
            joints = [model.joint(side + n).id for n in
                      ('HipYaw', 'HipRoll', 'HipPitch', 'KneePitch', 'AnklePitch', 'AnkleRoll')]
            self.va.append(model.jnt_dofadr[joints])
        self.body_ids = [model.body(n).id for n in ('leftFoot', 'rightFoot')]
        self.steps = 0

    def transition(self, phase):
        self.phase, self.elapsed = phase, 0.
        self.begin = self.p.copy()
        self.initial_weights = self.weights.copy()
        center = self.feet.mean(axis=0)
        self.end = np.array([center[0] - .04, center[1], self.height])
        if phase == 'transfer':
            support = 1 - self.side
            self.end[:2] = [self.feet[support, 0] - .04,
                           center[1] + .8*(self.feet[support, 1] - center[1])]
        if phase == 'swing':
            self.destination = self.feet[self.side].copy()
            self.destination[0] = self.feet[1-self.side, 0] + .08

    def contact_forces(self, data):
        forces = np.zeros(2)
        force = np.zeros(6)
        for i, contact in enumerate(data.contact):
            bodies = self.m.geom_bodyid[contact.geom]
            if 0 not in bodies:
                continue
            mujoco.mj_contactForce(self.m, data, i, force)
            for side, body in enumerate(self.body_ids):
                if body in bodies:
                    forces[side] += force[0]
        return forces

    def control(self, data, walking=True):
        m, b = self.m, self.b
        mujoco.mj_forward(m, data)
        forces = self.contact_forces(data)
        self.elapsed += m.opt.timestep
        u = min(1., self.elapsed / (self.SWING if self.phase == 'swing' else self.TRANSFER))
        smooth = u*u*(3-2*u)
        self.active_swing = self.side if self.phase == 'swing' else -1
        if self.phase in ('crouch', 'transfer', 'stop'):
            self.p = self.begin + (self.end-self.begin)*smooth
            final = np.ones(12)
            if self.phase == 'transfer':
                final[6*self.side:6*self.side+6] = .001
            self.weights = self.initial_weights + (final-self.initial_weights)*smooth
            if u >= 1:
                if self.phase == 'transfer' and walking and forces[1-self.side] > 100:
                    self.transition('swing')
                elif self.phase != 'transfer' and walking:
                    self.transition('transfer')
                elif not walking:
                    self.transition('hold' if self.phase == 'stop' else 'stop')
        elif self.phase == 'swing':
            self.fs[self.side] = self.feet[self.side] + (self.destination-self.feet[self.side])*smooth
            self.fs[self.side, 2] += .035*np.sin(np.pi*u)**2
            if u >= 1 and forces[self.side] > 1:
                self.feet[self.side] = self.destination
                self.fs = self.feet.copy()
                self.steps += 1
                self.side = 1-self.side
                self.transition('transfer' if walking else 'stop')
        elif walking:
            self.transition('transfer')

        for i, foot in enumerate(b.feet):
            mujoco.mj_jacSite(m, data, self.J[6*i:6*i+3], self.J[6*i+3:6*i+6], foot)
        acceleration = np.zeros(m.nv)
        acceleration[b.va] = 100*(b.home-data.qpos[b.qa])-20*data.qvel[b.va]
        acceleration[:3] = 100*(self.p-data.qpos[:3])-20*data.qvel[:3]
        acceleration[3:6] = -200*data.qpos[4:7]-20*data.qvel[3:6]
        for i, foot in enumerate(b.feet):
            jac = self.J[6*i:6*i+6]
            R = data.site_xmat[foot].reshape(3, 3)
            error = np.r_[self.fs[i]-data.site_xpos[foot], .5*np.array(
                [R[1, 2]-R[2, 1], R[2, 0]-R[0, 2], R[0, 1]-R[1, 0]])]
            desired = 200*error-30*jac@data.qvel
            if self.old_jac is not None:
                desired -= (jac-self.old_jac[6*i:6*i+6])@data.qvel/m.opt.timestep
            acceleration[self.va[i]] = np.linalg.solve(jac[:, self.va[i]]+np.eye(6)*1e-8,
                                                       desired-jac[:, :6]@acceleration[:6])
        self.old_jac = self.J.copy()
        rhs = np.zeros(m.nv)
        mujoco.mj_mulM(m, data, rhs, acceleration)
        rhs += data.qfrc_bias
        A = self.J[:, :6].T
        wrench = self.weights*(A.T@np.linalg.solve((A*self.weights)@A.T, rhs[:6]))
        torque = (rhs-self.J.T@wrench)[b.va]
        data.ctrl[:] = np.clip(torque, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0
