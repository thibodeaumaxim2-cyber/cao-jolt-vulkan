#include "editor/EquilibriumPolicy.hpp"
#include <cassert>

int main() {
  FastEquilibriumPolicy policy;
  auto decision = policy.update({0.85f, 0.05f, 1.0f});
  assert(decision.score >= 0.90f && decision.walkingAllowed);

  decision = policy.update({0.70f, 1.10f, 0.50f});
  assert(decision.score < 0.88f && !decision.walkingAllowed);

  decision = policy.update({0.76f, 0.10f, 0.90f});
  assert(!decision.walkingAllowed); // marginal balance must not chatter

  decision = policy.update({0.85f, 0.05f, 1.0f});
  assert(decision.walkingAllowed);
  return 0;
}
