#!/usr/bin/env python3
"""Torque-limited 29-DoF G1 neutral stand controller and regression test."""
from pathlib import Path
import mujoco, numpy as np

root=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(root/'assets/unitree_g1/g1_29dof.xml'))
d=mujoco.MjData(m); mujoco.mj_resetData(m,d)
# Unitree G1 joint order: both 6-DoF legs, 3-DoF waist, both 7-DoF arms.
targets=np.zeros(29)
# G1 actuator order is hip-pitch, hip-roll, hip-yaw, knee,
# ankle-pitch, ankle-roll for each leg. Keep the shins slightly flexed and
# match ankle pitch to the thigh so both soles load the ground.
targets[[0,3,4,6,9,10]]=[-.312,.669,-.363,-.312,.669,-.363]
targets[[15,16,17,18,19,20,21,22,23,24,25,26,27,28]]=[.2,.2,0,.6,.2,-.2,0,.6,.15,0,0,-.15,0,0]
targets[[12,13,14]]=0.0
# Unitree's published G1 deployment gains (leg motors, then waist/arms).
kp=np.array([100,100,100,150,40,40,100,100,100,150,40,40,
             300,300,300,100,100,50,50,20,20,20,100,100,50,50,20,20,20],dtype=float)
kd=np.array([2,2,2,4,2,2,2,2,2,4,2,2,
             3,3,3,2,2,2,2,1,1,1,2,2,2,2,1,1,1],dtype=float)
qadr=[]; dadr=[]
for a in range(29):
    j=m.actuator_trnid[a,0]; qadr.append(m.jnt_qposadr[j]); dadr.append(m.jnt_dofadr[j])
d.qpos[2]=.76
d.qpos[qadr]=targets
mujoco.mj_forward(m,d)
initial=float(d.qpos[2]); lowest=initial
start=targets.copy()
for step in range(2400):
    # Blend in the nominal pose over 1.5 seconds; this prevents a torque step
    # from throwing the free base before the feet have loaded.
    blend=min(1.0, step/(1.5/0.002))
    commanded=(1.0-blend)*d.qpos[qadr]+blend*start
    q=d.qpos[qadr]; v=d.qvel[dadr]
    tau=kp*(commanded-q)-kd*v
    # An unlimited MuJoCo motor has a placeholder [0, 0] ctrlrange.  Treat it
    # as unlimited; clipping it would silently disable every G1 motor.
    for actuator, value in enumerate(tau):
        if m.actuator_ctrllimited[actuator]:
            value=np.clip(value,m.actuator_ctrlrange[actuator,0],m.actuator_ctrlrange[actuator,1])
        d.ctrl[actuator]=value
    mujoco.mj_step(m,d); lowest=min(lowest,float(d.qpos[2]))
print(f'initial_height_m={initial:.3f} min_height_m={lowest:.3f}')
if lowest < .55: raise SystemExit('G1 neutral stand failed: pelvis fell below 0.55 m')
