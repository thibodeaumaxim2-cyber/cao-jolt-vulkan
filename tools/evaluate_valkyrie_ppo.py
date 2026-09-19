#!/usr/bin/env python3
"""Reject Valkyrie PPO checkpoints that do not remain grounded and upright."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("valkyrie_train", ROOT / "tools" / "train_valkyrie_ppo.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--steps", type=int, default=1200)
    parser.add_argument("--walking", action="store_true")
    args = parser.parse_args()
    env = module.ValkyrieEnv(walking=args.walking)
    policy = PPO.load(args.model, env=env)
    falls, height_falls, tilt_falls, distances, lows, lowest_upright, double_support_samples = 0, 0, 0, [], [], [], 0
    for seed in range(args.episodes):
        observation, _ = env.reset(seed=seed)
        initial_x, low, low_upright = float(env.d.qpos[0]), float(env.d.qpos[2]), 1.0
        for _ in range(args.steps):
            action, _ = policy.predict(observation, deterministic=True)
            observation, _, terminated, _, info = env.step(action)
            low = min(low, float(env.d.qpos[2]))
            low_upright = min(low_upright, float(info["upright"]))
            double_support_samples += int(info["double_support"])
            if terminated:
                falls += 1
                height_falls += int(info["height_failure"])
                tilt_falls += int(info["tilt_failure"])
                break
        distances.append(float(env.d.qpos[0]) - initial_x)
        lows.append(low)
        lowest_upright.append(low_upright)
    print(f"episodes={args.episodes} falls={falls} height_falls={height_falls} tilt_falls={tilt_falls} double_support_samples={double_support_samples} "
          f"min_height_m={min(lows):.3f} min_upright={min(lowest_upright):.3f} mean_forward_m={np.mean(distances):.3f}")
    if falls or min(lows) < env.min_height or (args.walking and np.mean(distances) <= .15):
        raise SystemExit("REJECTED: grounded Valkyrie policy regression failed")
    print("PASSED: candidate is eligible for separate native-runtime review")


if __name__ == "__main__":
    main()
