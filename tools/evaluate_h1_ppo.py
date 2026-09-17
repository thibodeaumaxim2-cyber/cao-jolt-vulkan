#!/usr/bin/env python3
"""Standing/fall regression for a PPO checkpoint; never promotes it to runtime."""
import argparse, importlib.util
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("h1train",ROOT/"tools/train_h1_ppo.py")
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)

def main():
    p=argparse.ArgumentParser(); p.add_argument("--model",type=Path,default=ROOT/"artifacts/h1_fullbody_ppo.zip"); p.add_argument("--episodes",type=int,default=20); p.add_argument("--steps",type=int,default=1200)
    a=p.parse_args(); env=module.H1Env(); policy=PPO.load(a.model,env=env)
    falls=0; distances=[]; heights=[]; survival=[]
    for seed in range(a.episodes):
        obs,_=env.reset(seed=seed); initial=float(env.d.qpos[0]); low=float(env.d.qpos[2])
        for step in range(a.steps):
            action,_=policy.predict(obs,deterministic=True); obs,_,terminated,_,_=env.step(action); low=min(low,float(env.d.qpos[2]))
            if terminated: falls+=1; break
        distances.append(float(env.d.qpos[0])-initial); heights.append(low); survival.append(step+1)
    print(f"episodes={a.episodes} falls={falls} fall_rate={falls/a.episodes:.1%} "
          f"mean_forward_m={np.mean(distances):.3f} min_height_m={min(heights):.3f} mean_survival_steps={np.mean(survival):.1f}")
    # Must hold a 5 s episode (1200 50-Hz actions) in every deterministic seed.
    if falls or min(heights)<.70: raise SystemExit("REJECTED: PPO policy fails standing/fall regression")
    print("PASSED: candidate may proceed to a separate integration review")
if __name__=="__main__": main()
