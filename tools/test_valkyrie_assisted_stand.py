#!/usr/bin/env python3
"""Five-second safety-fixture standing regression for the Valkyrie candidate."""
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from valkyrie_stand_controller import ValkyrieStandController


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/valkyrie/valkyrie.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    controller = ValkyrieStandController(model, safety_fixture=True)
    lowest_height = float(data.qpos[2])
    maximum_drift = 0.0
    for step in range(2500):
        controller.step(data)
        if 250 <= step < 300:
            data.qfrc_applied[0] += 300.0  # 100 ms bounded lateral disturbance
        mujoco.mj_step(model, data)
        lowest_height = min(lowest_height, float(data.qpos[2]))
        maximum_drift = max(maximum_drift, abs(float(data.qpos[0])))
    if not np.isfinite(data.qpos).all() or lowest_height < 1.05 or maximum_drift > 0.05:
        raise SystemExit("Valkyrie assisted standing regression failed")
    print(
        f"valkyrie_assisted_min_height_m={lowest_height:.3f} "
        f"max_lateral_drift_m={maximum_drift:.3f}"
    )


if __name__ == "__main__":
    main()
