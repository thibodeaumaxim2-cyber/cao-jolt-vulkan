#!/usr/bin/env python3
"""Torque-limited 29-DoF G1 neutral stand controller and regression test."""
from pathlib import Path
import mujoco, numpy as np

root=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(root/'assets/unitree_g1/scene.xml'))
d=mujoco.MjData(m); mujoco.mj_resetData(m,d)
# Unitree G1 joint order: both 6-DoF legs, 3-DoF waist, both 7-DoF arms.
targets=np.zeros(29); targets[[0,3,6,9]]=[-.35,.70,-.35,.70]
targets[[12,13,14]]=0.0
kp=np.array([115]*12+[65]*3+[28]*14,dtype=float)
kd=np.array([8]*12+[5]*3+[2.5]*14,dtype=float)
qadr=[]; dadr=[]
for a in range(29):
    j=m.actuator_trnid[a,0]; qadr.append(m.jnt_qposadr[j]); dadr.append(m.jnt_dofadr[j])
initial=float(d.qpos[2]); lowest=initial
for _ in range(2400):
    q=d.qpos[qadr]; v=d.qvel[dadr]
    tau=kp*(targets-q)-kd*v
    d.ctrl[:29]=np.clip(tau,m.actuator_ctrlrange[:29,0],m.actuator_ctrlrange[:29,1])
    mujoco.mj_step(m,d); lowest=min(lowest,float(d.qpos[2]))
print(f'initial_height_m={initial:.3f} min_height_m={lowest:.3f}')
if lowest < .55: raise SystemExit('G1 neutral stand failed: pelvis fell below 0.55 m')
