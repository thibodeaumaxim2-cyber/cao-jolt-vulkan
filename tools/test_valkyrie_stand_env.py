#!/usr/bin/env python3
"""Physical standing regression; zero policy action must sustain support."""

from pathlib import Path
import sys
import argparse

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from python.envs.valkyrie_stand_env import ValkyrieStandEnv


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=5)
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    environment = ValkyrieStandEnv(ROOT / "assets" / "valkyrie" / "valkyrie.xml")
    for seed in range(args.episodes):
        observation, _ = environment.reset(seed=seed)
        height = environment.target_pelvis_height
        min_height, min_upright, supported = height, 1., 0
        steps = round(args.seconds / environment.dt_policy)
        for _ in range(steps):
            observation, reward, done, _, info = environment.step(np.zeros(12, dtype=np.float32))
            assert environment.observation_space.contains(observation)
            assert np.isfinite(reward) and not done, (seed, info)
            assert not np.any(environment.data.qfrc_applied)
            assert not np.any(environment.data.xfrc_applied)
            min_height = min(min_height, info['pelvis_height'])
            min_upright = min(min_upright, info['upright'])
            supported += info['left_foot_force'] > 100 and info['right_foot_force'] > 100
        assert min_height > height - .05, (seed, min_height)
        assert min_upright > .98, (seed, min_upright)
        assert supported / steps > .95, (seed, supported / steps)
        assert np.linalg.norm(environment.data.qpos[:2]) < .10
        print(f'seed={seed} seconds={args.seconds:g} min_height={min_height:.6f} '
              f'min_upright={min_upright:.6f} support={supported/steps:.3f}', flush=True)


if __name__ == "__main__":
    main()
