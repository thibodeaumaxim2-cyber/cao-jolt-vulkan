# NASA Valkyrie (R5) asset preparation

The source description package is the pinned `third_party/val_description`
submodule, published under the NASA Open Source Agreement 1.3. Run:

```bash
.venv/bin/pip install pycollada
.venv/bin/python tools/prepare_valkyrie_assets.py
```

This generates binary STL visual meshes and `mesh_manifest.json` in this
directory. Build the isolated articulation with:

```bash
.venv/bin/python tools/export_valkyrie_urdf.py
.venv/bin/python tools/build_valkyrie_mujoco.py
.venv/bin/python tools/test_valkyrie_controller.py
.venv/bin/python tools/test_valkyrie_assisted_stand.py
.venv/bin/python tools/simulate_valkyrie.py --mode assisted --steps 2500
```

`valkyrie_sim.urdf` is the canonical expanded Xacro model; `valkyrie_mujoco.urdf`
removes only massless sensor frames that MuJoCo cannot represent. `valkyrie.xml`
is the separate 58-torque-actuator candidate with visual meshes and foot-box
collision proxies. The controller regression validates bounded actuator mapping
and PD recovery from safe joint perturbations in zero gravity.

It does **not** add Valkyrie to the live H1 simulator. A free-standing balance
controller has not passed yet, so this candidate must stay isolated until that
regression is added and succeeds. The assisted-standing regression is a
calibration safety fixture: it uses foot contacts for vertical support and a
bounded virtual horizontal/attitude gantry to protect the model during
controller development. It is intentionally not a free-standing result.

The generated model now exposes pelvis orientation, angular velocity, linear
acceleration, and left/right sole contact sensors. Use
`tools/simulate_valkyrie.py --mode free` to measure the unassisted baseline;
it is expected to report a fall until the contact-aware balance policy passes.

## Grounded policy workflow

`tools/train_valkyrie_ppo.py` trains a separate PPO candidate using only the
12 leg motors, pelvis IMU state, joint feedback, and the two sole contacts.
It clears every externally applied base force before each physics step. Start
with a standing policy, then a walking policy:

```bash
.venv/bin/python tools/optimize_valkyrie_stance.py --iterations 160
.venv/bin/python tools/train_valkyrie_ppo.py --workers 8 --steps 1000000 --output artifacts/valkyrie_stand_ppo
.venv/bin/python tools/evaluate_valkyrie_ppo.py --model artifacts/valkyrie_stand_ppo.zip
.venv/bin/python tools/train_valkyrie_ppo.py --walking --workers 8 --steps 3000000 --init-model artifacts/valkyrie_stand_ppo.zip --output artifacts/valkyrie_walk_ppo
.venv/bin/python tools/evaluate_valkyrie_ppo.py --walking --model artifacts/valkyrie_walk_ppo.zip
```

Only a checkpoint that passes the grounded evaluation (no falls and measurable
forward progress) may be considered for Vulkan integration.

## IK gait targets

`tools/valkyrie_ik.py` contains a DLS IK solver and `ValkyrieGaitSolver`,
which generates alternating left/right sole trajectories and bounded targets
for the 12 leg motors. Verify its kinematic contract with:

```bash
.venv/bin/python tools/test_valkyrie_gait_solver.py
```

These are motor targets, not a balance policy; runtime integration remains
gated on a grounded contact/balance regression.
