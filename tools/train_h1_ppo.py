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
import json
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "assets/unitree_h1/scene_full.xml"
WEIGHTS = ROOT / "assets/unitree_h1/weights.json"
JOINTS = ["left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint", "left_ankle_joint",
          "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "right_ankle_joint",
          "torso_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
          "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"]

class H1Env(gym.Env):
    metadata = {"render_modes": []}
    def __init__(self, walking=False):
        self.m = mujoco.MjModel.from_xml_path(str(XML)); self.d = mujoco.MjData(self.m)
        self.ids = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS])
        self.qa, self.da = self.m.jnt_qposadr[self.ids], self.m.jnt_dofadr[self.ids]
        self.aids = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINTS])
        self.home = np.array([0,0,-.1,.3,-.2,0,0,-.1,.3,-.2,0,.18,.05,0,.45,.18,-.05,0,.45],np.float32)
        # PPO starts as a residual stabilizer around the verified neutral
        # controller. Large direct actions make every fresh policy fall before
        # it collects a useful rollout; expand this only after stand passes.
        self.walking = walking
        self.action_scale = np.array([.018]*10+[.010]+[.014]*8,np.float32)
        self.base_action = np.zeros(10,np.float32)
        w=json.loads(WEIGHTS.read_text())
        self.wi=np.asarray(w['memory.weight_ih_l0'],np.float32); self.wh=np.asarray(w['memory.weight_hh_l0'],np.float32)
        self.bi=np.asarray(w['memory.bias_ih_l0'],np.float32)+np.asarray(w['memory.bias_hh_l0'],np.float32)
        self.w0=np.asarray(w['actor.0.weight'],np.float32); self.b0=np.asarray(w['actor.0.bias'],np.float32)
        self.w2=np.asarray(w['actor.2.weight'],np.float32); self.b2=np.asarray(w['actor.2.bias'],np.float32)
        self.h=np.zeros(64,np.float32); self.c=np.zeros(64,np.float32); self.policy_steps=0
        self.last_action = np.zeros(len(JOINTS),np.float32)
        self.action_space = spaces.Box(-1,1,(len(JOINTS),),np.float32)
        self.observation_space = spaces.Box(-np.inf,np.inf,(3+3+len(JOINTS)*2+10,),np.float32)
    def obs(self):
        q=self.d.qpos[3:7]; w,x,y,z=q
        gravity=np.array([2*(-z*x+w*y),-2*(z*y+w*x),1-2*(w*w+z*z)])
        x=np.zeros(41,np.float32); x[:3]=self.d.qvel[3:6]*.25; x[3:6]=gravity
        x[6:9]=[.35 if self.walking else 0.,0.,0.]
        x[9:19]=self.d.qpos[self.qa[:10]]-self.home[:10]; x[19:29]=self.d.qvel[self.da[:10]]*.05
        x[29:39]=self.base_action; phase=self.policy_steps*.002/.80; x[39:]=[np.sin(2*np.pi*phase),np.cos(2*np.pi*phase)]
        gates=self.bi+self.wi@x+self.wh@self.h
        i=1/(1+np.exp(-gates[:64])); f=1/(1+np.exp(-gates[64:128])); g=np.tanh(gates[128:192]); o=1/(1+np.exp(-gates[192:]) )
        self.c=f*self.c+i*g; self.h=o*np.tanh(self.c)
        a=self.w0@self.h+self.b0; a=np.where(a<0,np.expm1(a),a)
        # Keep the residual-training sandbox inside the verified actuator
        # envelope; the native deployment also applies a bounded target.
        self.base_action=np.clip(self.w2@a+self.b2,-1.0,1.0)
        return self._ppo_obs()
    def _ppo_obs(self):
        q=self.d.qpos[3:7]; w,x,y,z=q
        gravity=np.array([2*(-z*x+w*y),-2*(z*y+w*x),1-2*(w*w+z*z)])
        return np.r_[self.d.qvel[3:6]*.25,gravity,self.d.qpos[self.qa]-self.home,self.d.qvel[self.da]*.05,self.base_action].astype(np.float32)
    def reset(self, seed=None, options=None):
        super().reset(seed=seed); mujoco.mj_resetDataKeyframe(self.m,self.d,0)
        self.d.qpos[self.qa] = self.home + self.np_random.normal(0,.006,len(self.qa)); self.last_action.fill(0)
        self.base_action.fill(0); self.h.fill(0); self.c.fill(0); self.policy_steps=0; mujoco.mj_forward(self.m,self.d)
        return self._ppo_obs(), {}
    def step(self, action):
        before=float(self.d.qpos[0]); residual=np.clip(np.asarray(action),-1,1)*self.action_scale
        target=self.home.copy(); target[:10]+=self.base_action*.25+residual[:10]; target[10:]+=residual[10:]
        for _ in range(10):
            kp=np.array([150,150,150,200,40,150,150,150,200,40]+[55]*9)
            kd=np.array([2,2,2,4,2,2,2,2,4,2]+[7]*9)
            tau=kp*(target-self.d.qpos[self.qa])-kd*self.d.qvel[self.da]
            self.d.ctrl[self.aids]=np.clip(tau,self.m.actuator_ctrlrange[self.aids,0],self.m.actuator_ctrlrange[self.aids,1]); mujoco.mj_step(self.m,self.d)
            self.policy_steps+=1
        height=float(self.d.qpos[2]); forward=float(self.d.qpos[0])-before
        upright=float(self.d.xmat[1,8]); energy=float(np.mean(np.square(self.d.ctrl[self.aids])))
        terminated=height<.70 or abs(self.d.xmat[1,8])<.45
        # Curriculum: survival and uprightness dominate until the policy has
        # learned double support; progress then becomes profitable.
        reward=(3.0*forward if self.walking else 0) + 2.0*min(1,height/.98) + 1.5*max(0,upright) - .00005*energy - .02*np.mean(np.square(action)) - (10 if terminated else 0)
        return self.obs(), reward, terminated, False, {"height":height,"forward":forward}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--steps",type=int,default=200_000); ap.add_argument("--output",type=Path,default=ROOT/"artifacts/h1_fullbody_ppo"); ap.add_argument("--walking",action="store_true")
    a=ap.parse_args(); a.output.parent.mkdir(parents=True,exist_ok=True)
    model=PPO("MlpPolicy",H1Env(a.walking),verbose=1,n_steps=1024,batch_size=256,learning_rate=3e-4,policy_kwargs={"net_arch":[128,128]})
    model.learn(a.steps); model.save(a.output); print(f"saved {a.output}.zip")
if __name__ == "__main__": main()
