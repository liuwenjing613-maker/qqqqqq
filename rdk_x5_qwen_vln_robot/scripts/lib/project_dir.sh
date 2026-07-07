#!/usr/bin/env bash
# Resolve this Qwen-only project root.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Read-only reference to original robot repo for camera/lidar launch scripts (no modifications).
RDK_ORIGINAL_ROOT="${RDK_ORIGINAL_ROOT:-/root/rdk_x5_vln_robot}"
