#pragma once

enum class RendererLifecycle {
  Uninitialized,
  DeviceReady,
  SwapchainReady,
  PipelineReady,
  Recording,
  ShuttingDown
};

inline bool canRecord(RendererLifecycle state) {
  return state == RendererLifecycle::PipelineReady ||
         state == RendererLifecycle::Recording;
}
