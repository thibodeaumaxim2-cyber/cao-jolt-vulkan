#pragma once

#include <cstdint>

enum class SimulationEventKind { BodyAdded, BodyRemoved, BodySleeping, BodyWoke, ConstraintBroken };

struct SimulationEvent {
  SimulationEventKind kind = SimulationEventKind::BodyAdded;
  uint32_t objectId = 0;
};
