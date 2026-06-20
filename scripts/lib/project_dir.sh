#!/usr/bin/env bash
# Resolve repository root (rdk_x5_vln_robot) from scripts/lib/.
if [ -z "${PROJECT_DIR:-}" ]; then
  _lib_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  PROJECT_DIR="$(cd "$_lib_dir/../.." && pwd)"
fi
export PROJECT_DIR
