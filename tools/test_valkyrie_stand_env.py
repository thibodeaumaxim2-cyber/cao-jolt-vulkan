#!/usr/bin/env python3
"""Focused smoke test for the motor-only Valkyrie standing environment."""

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from python.envs.valkyrie_stand_env import ValkyrieStandEnv


def main() -> None:
    environment = ValkyrieStandEnv(ROOT / "assets" / "valkyrie" / "valkyrie.xml")
    observation, _ = environment.reset(seed=7)
    assert observation.shape == (48,), observation.shape
    _, _, _, _, info = environment.step(np.zeros(12, dtype=np.float32))
    assert np.isfinite(info["pelvis_height"])
    assert np.isfinite(info["upright"])
    print(f"valkyrie_stand_env_observation={observation.shape[0]} timestep_s={environment.dt_sim:g}")


if __name__ == "__main__":
    main()
