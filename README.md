# CAO MuJoCo Vulkan

Standalone C++20/Vulkan replacement for the portfolio CAO 3D physics app. It has no PHP, browser, CDN, Three.js, or Rapier dependency.

## Features

- MuJoCo articulated dynamics, contact solving, position actuators, friction, and a static ground plane.
- Six-leg robot with 24 rotary actuators and a balance-gated one-foot crawl gait.
- Official Unitree H1 MuJoCo model import, including the upstream STL assets and inertial data.
- Fast equilibrium policy that pauses walking on instability and resumes only after recovery.
- Box, cylinder, sphere and beam creation.
- Mouse selection, object tree, editable transform/property panel, translate/rotate/scale modes, delete and snap-to-0.25m.
- Perspective orbit camera, ISO/top/front/right presets and frame-all.
- Configurable 2–12-level cube pyramid, paused Build and falling Demo.
- Space/play/pause, automatic demo restart after the pyramid falls 50m or the world is inactive for 7 seconds.
- JSON scene export/import and New scene.
- Minimal dependency footprint: GLFW + Vulkan + Dear ImGui + nlohmann/json + MuJoCo.

## Build

Install CMake 3.20+, a C++20 compiler, Vulkan development files (including `glslc`), and GLFW prerequisites. MuJoCo, GLFW, Dear ImGui, and nlohmann/json are fetched and compiled automatically by default. Then:

```bash
cmake -S . -B build
cmake --build build --config Release -j
./build/cao-jolt
```

The H1 model is an upstream Git submodule. Initialize it after a fresh clone:

```bash
git submodule update --init --recursive
```

It is sourced from Unitree's BSD-3-Clause-licensed
[`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) repository.

The first configure downloads the pinned MuJoCo release from its official repository and builds it as part of this project.

On Windows, use the generated `Release/cao-jolt.exe`. If the machine is offline, pre-populate CMake's FetchContent cache or configure on a machine with network access first.

Run a short deterministic gait check after building:

```bash
./build/cao-headless walk --quick
```

Omit `--quick` to run the complete 10-candidate, 60-second controller search.

## Controls

Left click selects. Right drag orbits. Middle drag pans. Wheel zooms. `W`, `E`, `R` select transform modes. `Space` toggles simulation. `Delete` removes the selection. `N` creates a box. `Ctrl+S` exports `scene.json`; `Ctrl+O` imports it. The on-screen toolbar documents the remaining actions.

## MuJoCo integration

`src/editor/MuJoCoBridge.cpp` creates an in-memory MJCF hexapod with a free torso, 24 constrained position actuators, high-friction feet, and a ground plane. The render scene mirrors MuJoCo geometry transforms each frame while `src/renderer/VulkanRenderer.cpp` owns the Vulkan presentation path. `cao-headless` provides a windowless simulation target for regression checks.

The **Import Unitree H1** toolbar action loads Unitree's original `scene.xml`
and mesh assets into MuJoCo. Vulkan loads and renders the matching binary STL
mesh for every visible H1 link, synchronized to MuJoCo body transforms. The
H1 uses a torque-safe bent-stance controller with pelvis roll/pitch equilibrium
assist and a slow, alternating assisted-step controller. It lifts and advances
one leg at a time, shifts toward the support side, and pauses the gait below an
88% equilibrium score. This is a simulation controller, not a hardware-ready
locomotion policy.

**H1 learning walk** adds a bounded online learning layer. It tests three
nearby conservative step profiles, rewards forward progress while penalizing
loss of equilibrium, and selects the best observed profile. Torque limits and
the 88% balance gate remain hard safety constraints.

## PyTorch locomotion policy

**Recommended H1 mode: H1 Unitree pretrained walk.** Import H1 using the
Unitree walk button (or select that motion script for an existing H1). This
loads Unitree's matching ten-joint deployment model and recurrent walking
policy. Selecting the mode rebuilds the physics model on its next step.
The upper body is fixed as in the training model. Vulkan remains the renderer.
Forward velocity command is currently capped at 0.5 m/s.

The default H1 scene is a navigation course: six movable pyramid boxes, ten
fixed barriers, and a green circular goal at `(6.2 m, 0 m)`. A native A*
planner builds a 25 cm grid route around inflated obstacle cells and sends the
next waypoint as forward, lateral, and yaw commands to the Unitree policy.
The goal test requires a final distance of 60 cm or less; it does not fake the
robot's location or bypass physical collisions.

**Local macro AI:** `assets/navigation_macro_policy.json` is a small PyTorch-
trained 4→64→3 policy, run directly in C++ with no Python dependency at
runtime. It interprets the body-frame A* waypoint and gates the navigation
command; the deterministic A* vector remains authoritative for steering because
even small learned lateral errors accumulated into obstacle contacts in the
physical simulation. Re-export it after changing its teacher with
`.venv/bin/python tools/train_navigation_macro.py`. The current Unitree policy
has a rigid upper body and no arm joints, so saluting or torso-bending is shown
as unavailable rather than simulated deceptively; those actions require a
separate full-body H1 model and policy.

## Full-body goal policy

`tools/train_fullbody_goal_policy.py` trains and exports
`assets/fullbody_goal_policy.json` from randomized states of the full Unitree
H1 MuJoCo model. It accepts 19 joint positions, 19 joint velocities, and one
of four goals (`neutral`, `kneel_left`, `kneel_right`, `salute`), then predicts
19 safe joint targets. This is a behaviour-cloning bootstrap, not a claim of
hardware-ready reinforcement learning: its targets are conservative poses and
the next phase is reward-based fine-tuning with contact, balance, energy, and
goal-distance rewards.

```bash
.venv/bin/python tools/train_fullbody_goal_policy.py
```

Run the eight-second contact-based regression with
`./build/cao-headless h1unitree --quick`, or a 60-second run without
`--quick`. Successful steps require loss of ground contact, at least 2 cm
ankle rise from the preceding contact height, and renewed contact at least
5 cm forward. Both feet must complete at least two such steps.
This mode bypasses the earlier virtual pelvis forces and does not consume
the FBX approximation. See `assets/unitree_h1/README.md` for provenance.

Validation correction: the earlier foot-clearance metric was relative to the
pelvis and could count crouching as foot lift. It now measures world-height
change. Earlier reported 10–17 cm clearances do not establish real steps.
The controller also retains external pelvis balance assistance; passing a
smoke test does not establish unassisted physical walking.

`cao-headless h1contact --quick` exercises the experimental contact/IK
controller (script 4). It holds sagittal stance targets in world space and
requires airborne motion followed by ground contact before switching feet.
Its first trial failed during support transfer; it is not the default and is
not yet a validated walk or a full FBX pose retargeter.

`tools/train_h1_policy.py` trains a small PyTorch imitation policy from the
bundled Mesh2Motion CC0 walk timing and writes `assets/h1_locomotion_policy.json`.
`MuJoCoBridge` loads that file automatically when H1 learning walk starts and
runs its linear inference natively in C++; the application does not need a
Python interpreter at runtime.

```bash
python3 -m venv .venv
.venv/bin/pip install torch
blender --background assets/motion/mesh2motion-human-walk-large.blend \
  --python tools/export_mesh2motion_fbx.py -- assets/motion/mesh2motion-walk.fbx
blender --background --python-expr "import bpy; bpy.ops.import_scene.fbx(filepath='assets/motion/mesh2motion-walk.fbx')" \
  --python tools/extract_mesh2motion_walk.py -- assets/motion/mesh2motion_walk.json
.venv/bin/python tools/train_h1_policy.py
python3 tools/retarget_mesh2motion_to_h1.py
cmake --build build -j 4
```

The generated `assets/h1_motion_reference.json` is also loaded by **H1 learning
walk**. It is a direct, phase-by-phase visual retarget of the source walk,
bounded to H1's joint range. It does not replay an FBX rigidly: MuJoCo keeps
the feet in contact and rejects phases that violate the balance gate.

The initial policy is intentionally supervised and conservative. The next
iteration can replace its teacher trajectories with rollouts and rewards from
the headless MuJoCo simulator for reinforcement learning.

The project includes the CC0 Mesh2Motion large-walk source at
`assets/motion/mesh2motion-human-walk-large.blend`, with its provenance and
license recorded in `assets/motion/README.md`. It is the motion-imitation
reference for retargeting human hip, knee, ankle, and shoulder timing to H1.

`H1 30 cm ZMP walk (experimental)` is the start of the next controller layer:
it has a continuous support/swing trajectory and a bounded COM-over-support
force. The headless test measures ground-relative ankle placement and currently
rejects it as below the 25 cm acceptance threshold. A real 30 cm gait remains
the next milestone, requiring contact-state estimation, inverse kinematics, and
a whole-body MPC/ZMP solver.

## Project status 

The project is an active prototype moving toward a reusable native physics editor. The Vulkan path is the supported interactive renderer; the old OpenGL wording has been removed because there is currently no OpenGL application entry point in this repository.

See [docs/ROADMAP.md](docs/ROADMAP.md) for the recommended development sequence.
