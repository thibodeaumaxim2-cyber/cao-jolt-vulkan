# Unitree H1 pretrained locomotion

Source: https://github.com/unitreerobotics/unitree_rl_gym
Revision: 276801e46c5d433564f24658bac64f254b7d2d4b
License: upstream BSD-3-Clause, retained in LICENSE.

Imported unchanged:
- resources/robots/h1/h1.xml, scene.xml, meshes/
- deploy/pre_train/h1/motion.pt

Deployment contract comes from deploy/deploy_mujoco/configs/h1.yaml and
deploy/deploy_mujoco/deploy_mujoco.py at that revision.

weights.json is exported from motion.pt by tools/export_unitree_policy.py.
inference_test.json contains deterministic multi-step TorchScript outputs
for checking the native recurrent implementation.

The model has ten moving leg joints and a rigid upper body. It is deliberately
paired with the trained policy; the earlier 19-joint H1 model is not compatible.
The policy is an LSTM (41 inputs, 64 hidden units) followed by a 32-unit ELU
actor and ten outputs. Hidden and cell states reset when the model is rebuilt.
Inference is 50 Hz; torque PD and physics run at 500 Hz.

The official mode uses original friction, model actuator force limits and
no external pelvis forces, hand-coded gait, or equilibrium-score phase gate.
It is not an FBX imitation policy and cannot reproduce a requested clip.

Validation on this checkout:
- Native inference versus 20 consecutive TorchScript evaluations: maximum
  absolute error 1.43e-6 (including recurrent state).
- 60 simulated seconds: 25.08 m forward displacement, 74 left and 75 right
  contact-verified steps, minimum pelvis height 0.990 m, no fall detected.
- Maximum ankle rise above its preceding contact height: 0.0371 m.
- The Vulkan executable builds; these are physics/headless measurements,
  not a visual comparison with the source animation.

Re-export weights and parity fixtures with
`.venv/bin/python tools/export_unitree_policy.py`.
PyTorch is needed for this offline export only; runtime inference is native.
