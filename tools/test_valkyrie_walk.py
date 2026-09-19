#!/usr/bin/env python3
"""Regression for physical stepping, stopping and resuming on level ground."""
from pathlib import Path
import sys
import argparse
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from python.envs.valkyrie_balance import ValkyrieBalance
from python.envs.valkyrie_walk import ValkyrieWalk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=3)
    args = parser.parse_args()
    for seed in range(args.episodes):
        m = mujoco.MjModel.from_xml_path(str(ROOT/'assets/valkyrie/valkyrie.xml'))
        d = mujoco.MjData(m)
        balance = ValkyrieBalance(m)
        balance.reset(d)
        d.qpos[balance.qa] += np.random.default_rng(seed).uniform(-.002, .002, m.nu)
        for _ in range(2000):
            balance.control(d)
            mujoco.mj_step(m, d)
        walk = ValkyrieWalk(m, d, balance)
        old_contact = np.ones(2, dtype=bool)
        takeoff = d.site_xpos[balance.feet].copy()
        clearance = np.zeros(2)
        verified = np.zeros(2, dtype=int)
        min_height, min_upright, min_sole = 10., 1., 0.
        stop_position = None
        for step in range(60000):
            t = step * m.opt.timestep
            walking = t < 25 or 35 <= t < 50
            qpos, qvel = d.qpos.copy(), d.qvel.copy()
            walk.control(d, walking)
            assert np.array_equal(d.qpos, qpos) and np.array_equal(d.qvel, qvel)
            assert not np.any(d.qfrc_applied) and not np.any(d.xfrc_applied)
            assert np.isfinite(d.ctrl).all()
            mujoco.mj_step(m, d)
            min_height = min(min_height, d.qpos[2])
            min_upright = min(min_upright, d.xmat[1, 8])
            min_sole = min(min_sole, *d.site_xpos[balance.feet, 2])
            assert d.qpos[2] > 1.10 and d.xmat[1, 8] > .99, (seed, t, d.qpos[:3])
            assert abs(d.qpos[1]) < .20
            contacts = walk.contact_forces(d) > 1
            for side, site in enumerate(balance.feet):
                if old_contact[side] and not contacts[side]:
                    takeoff[side] = d.site_xpos[site]
                    clearance[side] = 0
                if not contacts[side]:
                    clearance[side] = max(clearance[side], d.site_xpos[site, 2]-takeoff[side, 2])
                if contacts[side] and not old_contact[side]:
                    if clearance[side] > .02 and d.site_xpos[site, 0]-takeoff[side, 0] > .05:
                        verified[side] += 1
            old_contact = contacts
            if step == 57000:
                stop_position = d.qpos[:3].copy()
        assert np.all(verified >= 3), (seed, verified)
        assert d.qpos[0] > .65, (seed, d.qpos[0])
        assert min_sole > -.005, (seed, min_sole)
        assert walk.phase == 'hold' and np.all(walk.contact_forces(d) > 100)
        assert np.linalg.norm(d.qpos[:3]-stop_position) < .005
        assert np.linalg.norm(d.qvel[:6]) < .025
        print(f'seed={seed} steps={verified.tolist()} forward={d.qpos[0]:.3f}m '
              f'min_height={min_height:.3f} upright={min_upright:.6f} stopped=yes', flush=True)


if __name__ == '__main__':
    main()
