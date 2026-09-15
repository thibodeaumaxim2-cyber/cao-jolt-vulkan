#pragma once

#include <algorithm>

struct EquilibriumInput {
  float torsoHeightM = 0.0f;
  float horizontalSpeedMps = 0.0f;
  float supportRatio = 0.0f;
};

struct EquilibriumDecision {
  float score = 0.0f;
  bool walkingAllowed = false;
};

// Tiny deterministic policy for the physics loop. Hysteresis stops walking
// quickly when balance is lost and resumes it only after a clear recovery.
class FastEquilibriumPolicy {
 public:
  EquilibriumDecision update(const EquilibriumInput &input) {
    // Low-profile hexapod: nominal crouched torso height is approximately
    // 0.85 m, and 0.70 m is the minimum safe recovery height.
    const float height = std::clamp((input.torsoHeightM - 0.70f) / 0.16f,
                                    0.0f, 1.0f);
    const float speed = std::clamp(1.0f - input.horizontalSpeedMps / 0.80f,
                                   0.0f, 1.0f);
    const float support = std::clamp(input.supportRatio, 0.0f, 1.0f);
    const float score = height * 0.55f + speed * 0.30f + support * 0.15f;

    // Walking is never permitted below 88%. Restart at 90% so a score that
    // hovers near the limit cannot rapidly toggle the gait on and off.
    if (walkingAllowed_) {
      if (score < 0.88f) walkingAllowed_ = false;
    } else if (score >= 0.90f) {
      walkingAllowed_ = true;
    }
    return {score, walkingAllowed_};
  }

  void reset(bool walkingAllowed = false) { walkingAllowed_ = walkingAllowed; }

 private:
  bool walkingAllowed_ = false;
};
