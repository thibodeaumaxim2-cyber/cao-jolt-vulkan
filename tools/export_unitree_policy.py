"""Export the official H1 TorchScript network for native C++ inference."""
import json
from pathlib import Path
import torch

root = Path(__file__).resolve().parents[1] / "assets/unitree_h1"
model = torch.jit.load(str(root / "motion.pt"), map_location="cpu").eval()
weights = {k: v.tolist() for k, v in model.state_dict().items()
           if k not in ("hidden_state", "cell_state")}
(root / "weights.json").write_text(json.dumps(weights))
model.reset_memory()
fixtures = []
torch.manual_seed(17)
for _ in range(20):
    obs = torch.randn(1, 41) * 0.2
    fixtures.append({"observation": obs[0].tolist(),
                     "action": model(obs).detach()[0].tolist()})
(root / "inference_test.json").write_text(json.dumps(fixtures))
