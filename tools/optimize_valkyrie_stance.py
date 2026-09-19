#!/usr/bin/env python3
"""Find a grounded, motor-only symmetric Valkyrie standing posture."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "assets" / "valkyrie" / "valkyrie.xml"
OUTPUT = ROOT / "assets" / "valkyrie" / "stance_targets.json"
JOINTS = ("HipYaw", "HipRoll", "HipPitch", "KneePitch", "AnklePitch", "AnkleRoll")
SIDES = ("left", "right")
# Conservative limits around the imported upright pose.  The search is not
# allowed to discover a crouch by pushing the pelvis through the floor.
BOUNDS = np.asarray(((-.10, .10), (-.10, .10), (-.45, .20), (.00, .95), (-.55, .30), (-.16, .16)))


def evaluate(model, targets, seconds):
    data = mujoco.MjData(model)
    data.qpos[2] = 1.16  # calibrated double-support contact height
    actuator_ids, qadr, dadr = [], [], []
    for side in SIDES:
        for joint in JOINTS:
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, side + joint)
            source_joint = model.actuator_trnid[actuator, 0]
            actuator_ids.append(actuator)
            qadr.append(model.jnt_qposadr[source_joint])
            dadr.append(model.jnt_dofadr[source_joint])
    actuator_ids, qadr, dadr = map(np.asarray, (actuator_ids, qadr, dadr))
    full_targets = np.tile(targets, 2)
    data.qpos[qadr] = full_targets
    mujoco.mj_forward(model, data)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    feet = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "leftFoot"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rightFoot")}
    low_height, low_upright, support = float(data.qpos[2]), 1.0, 0
    total_steps = int(seconds / model.opt.timestep)
    completed_steps = 0
    for _ in range(total_steps):
        kp = np.asarray((90, 110, 180, 220, 120, 95) * 2)
        kd = np.asarray((8, 10, 18, 20, 12, 10) * 2)
        torque = kp * (full_targets - data.qpos[qadr]) - kd * data.qvel[dadr]
        data.ctrl[actuator_ids] = np.clip(torque, model.actuator_ctrlrange[actuator_ids, 0], model.actuator_ctrlrange[actuator_ids, 1])
        data.qfrc_applied[:] = 0.0
        mujoco.mj_step(model, data)
        low_height = min(low_height, float(data.qpos[2]))
        low_upright = min(low_upright, float(data.xmat[pelvis, 8]))
        contact_bodies = {int(model.geom_bodyid[g]) for contact in data.contact for g in contact.geom}
        support += int(feet.issubset(contact_bodies))
        completed_steps += 1
        if low_height < .75 or low_upright < .25 or not np.isfinite(data.qpos).all():
            break
    # Survival dominates: a posture that collapses upright must never outrank
    # one that remains supported for the full rollout.
    survival = completed_steps / total_steps
    score = 40.0 * survival + 8.0 * min(low_height, 1.16) + 5.0 * low_upright + 2.0 * support / total_steps
    return score, low_height, low_upright, support, survival


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=160)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(XML))
    rng = np.random.default_rng(args.seed)
    best = np.zeros(6)
    best_score, best_height, best_upright, best_support, best_survival = evaluate(model, best, args.seconds)
    spread = np.asarray((.05, .05, .16, .24, .16, .07))
    for index in range(args.iterations):
        candidate = np.clip(best + rng.normal(0, spread), BOUNDS[:, 0], BOUNDS[:, 1])
        score, height, upright, support, survival = evaluate(model, candidate, args.seconds)
        if score > best_score:
            best, best_score = candidate, score
            best_height, best_upright, best_support, best_survival = height, upright, support, survival
            spread *= .92
            print(f"iteration={index + 1} score={score:.3f} survival={survival:.2%} height={height:.3f} upright={upright:.3f}")
    if best_survival < 0.999 or best_height < .95 or best_upright < .70:
        raise SystemExit("REJECTED: no four-second grounded stance found; do not train from this candidate")
    payload = {"format": "cao_valkyrie_stance_targets_v1", "joints": list(JOINTS),
               "symmetric_targets_rad": best.tolist(), "score": best_score,
               "min_pelvis_height_m": best_height, "min_upright": best_upright,
               "double_support_steps": best_support, "survival_ratio": best_survival, "duration_seconds": args.seconds}
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"saved {args.output} targets={np.round(best, 4).tolist()} score={best_score:.3f}")


if __name__ == "__main__":
    main()
