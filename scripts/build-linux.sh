#!/usr/bin/env bash
set -euo pipefail

build_dir="${1:-build}"
cmake -S . -B "$build_dir" -DCAO_RENDERER=VULKAN
cmake --build "$build_dir" --parallel "$(nproc)"
