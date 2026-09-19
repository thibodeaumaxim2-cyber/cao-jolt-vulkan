# Valkyrie standing

The Vulkan player and `train_valkyrie_ppo.py` standing mode now use full-body
joint feedback plus a double-support wrench estimate. The controller runs at
1 kHz. Feedforward subtracts the estimated sole load from the generalized bias
before applying the remaining joint torque through the 58 motors. The ground
reaction itself is computed by MuJoCo. No root force, pose override, or tether
is used during simulation. The starting pose is grounded once at reset.

Run the rebuilt player with `./build/cao-jolt --valkyrie`, or select NASA Valkyrie
in the robot library. Import starts the standing simulation.

For walking use `./build/cao-jolt --valkyrie-walk`, or select **Slow walk** in
the Motion script dropdown. Selecting **Motor-controlled stand** completes any
current swing, transfers back to double support, and holds the stopped pose.
The initial crouch and each weight transfer take two seconds; swings take
1.4 seconds with an 8 cm forward placement and 3.5 cm commanded clearance.
This is deliberately a slow level-ground gait, about 2 cm/s, not a fast walk.

Walking uses inverse dynamics with feedback on pelvis pose, foot position and
orientation, and measured ground contact. The support transfer is gradual;
the next step waits for touchdown. Joint torques obey the MJCF motor limits.
The Python reference is `valkyrie_walk.py`; the native implementation is
`src/editor/ValkyrieWalk.hpp`. Neither writes root state during motion.

Walking and stop/resume regressions:

```sh
./build/cao-headless valkyriewalk
./build/cao-headless valkyriewalkstop
OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/test_valkyrie_walk.py
```

Validate physical standing from the repository root:

```sh
.venv/bin/python tools/test_valkyrie_stand_env.py --episodes 20 --seconds 60
./build/cao-headless valkyrie
```

The Python regression checks height loss, upright orientation, measured support
on both feet, displacement, and absence of external applied forces. Its default
is five ten-second episodes. The native headless command runs 60 seconds.

Standing no longer requires PPO to learn basic support. To train small residual
leg-target corrections on top of it:

```sh
.venv/bin/python tools/train_valkyrie_ppo.py --workers 8 --steps 1000000 --output artifacts/valkyrie_stand_fullbody
.venv/bin/python tools/evaluate_valkyrie_ppo.py --model artifacts/valkyrie_stand_fullbody.zip
```

Start a fresh checkpoint: old 34-observation checkpoints are incompatible with
this 48-observation environment. The trainer's `--walking` flag still uses the
older experimental RL environment and cannot load this standing checkpoint
directly; it is separate from the deterministic walking controller in the
player. Current validation covers level ground and small reset perturbations,
not rough terrain or recovery from large pushes.
