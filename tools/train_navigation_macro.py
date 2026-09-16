#!/usr/bin/env python3
"""Train the local high-level navigation macro consumed by the C++ app."""
import json
from pathlib import Path
import torch

torch.manual_seed(23)
inputs = torch.empty(8192, 4).uniform_(-1.0, 1.0)
inputs[:, 3].uniform_(0.0, 1.0)
# Teacher: a cautious body-frame waypoint follower. Inputs x/y are the local
# waypoint position divided by 1.5 m.  Convert that vector back into a unit
# direction so a near waypoint receives the same deliberate walking command
# as a distant one; the macro never controls individual motors.
planar_norm = torch.clamp(torch.linalg.vector_norm(inputs[:, :2], dim=1), min=0.04)
target = torch.stack((
    torch.clamp(0.50 * inputs[:, 0] / planar_norm, -0.50, 0.50),
    torch.clamp(0.50 * inputs[:, 1] / planar_norm, -0.50, 0.50),
    torch.clamp(0.20 * 12.0 * inputs[:, 2], -0.20, 0.20),
), dim=1)
model = torch.nn.Sequential(torch.nn.Linear(4, 64), torch.nn.Tanh(), torch.nn.Linear(64, 3), torch.nn.Tanh())
optimizer = torch.optim.Adam(model.parameters(), lr=0.02)
for _ in range(2500):
    loss = torch.mean((model(inputs) - target) ** 2)
    optimizer.zero_grad(); loss.backward(); optimizer.step()
first, second = model[0], model[2]
payload = {"format": "cao_navigation_macro_mlp_v1", "inputs": ["local_x", "local_y", "yaw_error", "distance"],
           "outputs": ["forward_command", "lateral_command", "yaw_command"],
           "layer1_weight": first.weight.detach().tolist(), "layer1_bias": first.bias.detach().tolist(),
           "layer2_weight": second.weight.detach().tolist(), "layer2_bias": second.bias.detach().tolist()}
Path("assets/navigation_macro_policy.json").write_text(json.dumps(payload, indent=2) + "\n")
