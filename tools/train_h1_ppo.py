#!/usr/bin/env python3
"""Train a full-body H1 MuJoCo locomotion policy with PPO.

The checkpoint is intentionally kept separate from the C++ runtime policy.
Promote it only after a contact/standing regression has passed.
"""
import argparse
from pathlib import Path
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "assets/unitree_h1/scene_full.xml"
JOINTS = ["left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint", "left_ankle_joint",
          "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "right_ankle_joint",
          "torso_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
          "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"]

class H1Env(gym.Env):
    metadata = {"render_modes": []}
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(XML)); self.d = mujoco.MjData(self.m)
        self.ids = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS])
        self.qa, self.da = self.m.jnt_qposadr[self.ids], self.m.jnt_dofadr[self.ids]
        self.aids = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINTS])
        self.home = np.array([0,0,-.1,.3,-.2,0,0,-.1,.3,-.2,0,.18,.05,0,.45,.18,-.05,0,.45],np.float32)
        # PPO starts as a residual stabilizer around the verified neutral
        # controller. Large direct actions make every fresh policy fall before
        # it collects a useful rollout; expand this only after stand passes.
        self.action_scale = np.array([.025]*10+[.012]+[.018]*8,np.float32)
        self.last_action = np.zeros(len(JOINTS),np.float32)
        self.action_space = spaces.Box(-1,1,(len(JOINTS),),np.float32)
        self.observation_space = spaces.Box(-np.inf,np.inf,(3+3+len(JOINTS)*2,),np.float32)
    def obs(self):
        q=self.d.qpos[3:7]; w,x,y,z=q
        gravity=np.array([2*(-z*x+w*y),-2*(z*y+w*x),1-2*(w*w+z*z)])
        return np.r_[self.d.qvel[3:6]*.25,gravity,self.d.qpos[self.qa]-self.home,self.d.qvel[self.da]*.05].astype(np.float32)
    def reset(self, seed=None, options=None):
        super().reset(seed=seed); mujoco.mj_resetDataKeyframe(self.m,self.d,0)
        self.d.qpos[self.qa] = self.home + self.np_random.normal(0,.012,len(self.qa)); self.last_action.fill(0); mujoco.mj_forward(self.m,self.d)
        return self.obs(), {}
    def step(self, action):
        before=float(self.d.qpos[0]); self.last_action=.85*self.last_action+.15*np.asarray(action)
        target=self.home+self.last_action*self.action_scale
        for _ in range(10):
            tau=180*(target-self.d.qpos[self.qa])-12*self.d.qvel[self.da]
            self.d.ctrl[self.aids]=np.clip(tau,self.m.actuator_ctrlrange[self.aids,0],self.m.actuator_ctrlrange[self.aids,1]); mujoco.mj_step(self.m,self.d)
        height=float(self.d.qpos[2]); forward=float(self.d.qpos[0])-before
        upright=float(self.d.xmat[1,8]); energy=float(np.mean(np.square(self.d.ctrl[self.aids])))
        terminated=height<.70 or abs(self.d.xmat[1,8])<.45
        # Curriculum: survival and uprightness dominate until the policy has
        # learned double support; progress then becomes profitable.
        reward=2.5*forward + 1.5*min(1,height/.98) + 1.2*max(0,upright) - .00008*energy - (8 if terminated else 0)
        return self.obs(), reward, terminated, False, {"height":height,"forward":forward}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--steps",type=int,default=200_000); ap.add_argument("--output",type=Path,default=ROOT/"artifacts/h1_fullbody_ppo")
    a=ap.parse_args(); a.output.parent.mkdir(parents=True,exist_ok=True)
    model=PPO("MlpPolicy",H1Env(),verbose=1,n_steps=1024,batch_size=256,learning_rate=3e-4,policy_kwargs={"net_arch":[128,128]})
    model.learn(a.steps); model.save(a.output); print(f"saved {a.output}.zip")
if __name__ == "__main__": main()
