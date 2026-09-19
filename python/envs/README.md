# Valkyrie standing

The Vulkan player and `train_valkyrie_ppo.py` standing mode now use full-body
joint feedback plus a double-support wrench estimate. The controller runs at
1 kHz. Feedforward subtracts the estimated sole load from the generalized bias
before applying the remaining joint torque through the 58 motors. The ground
reaction itself is computed by MuJoCo. No root force, pose override, or tether
is used during simulation. The starting pose is grounded once at reset.

Run the rebuilt player with `./build/cao-jolt --valkyrie`, or select NASA Valkyrie
in the robot library. Import starts the standing simulation. Walking remains disabled.

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
this 48-observation environment. Walking still uses the experimental older
environment and cannot load this standing checkpoint directly. Standing tests
cover level ground and small reset perturbations, not walking, rough terrain,
or recovery from large pushes.
