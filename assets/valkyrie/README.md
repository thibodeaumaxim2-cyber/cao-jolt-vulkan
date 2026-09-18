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
