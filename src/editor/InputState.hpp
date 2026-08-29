#pragma once

struct InputState {
  bool leftMousePressed = false;
  bool leftMouseDown = false;
  bool rightMouseDown = false;
  bool middleMouseDown = false;
  bool shiftDown = false;
  bool ctrlDown = false;
  double cursorX = 0.0;
  double cursorY = 0.0;
  double cursorDeltaX = 0.0;
  double cursorDeltaY = 0.0;
  double scrollDelta = 0.0;
};
