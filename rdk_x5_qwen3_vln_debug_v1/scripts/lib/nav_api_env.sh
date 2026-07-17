#!/usr/bin/env bash

_nav_api_is_placeholder() {
  local value="${1:-}"
  case "$value" in
    ""|your_vision_model_name_here|your_dashscope_api_key_here|sk-your-key-here|sk-xxxxxxxxxxxxxxxx)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

# Load visual-navigation API settings without clobbering a valid voice key.
# Args: project_dir nav_root
load_nav_api_env() {
  local project_dir="$1"
  local nav_root="$2"
  local saved_key="${DASHSCOPE_API_KEY:-}"
  local env_file

  for env_file in \
    "$project_dir/.env" \
    "$nav_root/.env.local"; do
    if [[ -f "$env_file" ]]; then
      set -a
      # shellcheck disable=SC1090
      source "$env_file"
      set +a
    fi
  done

  if _nav_api_is_placeholder "${DASHSCOPE_API_KEY:-}" && [[ -n "$saved_key" ]]; then
    export DASHSCOPE_API_KEY="$saved_key"
  fi

  if _nav_api_is_placeholder "${QWEN_MODEL:-}"; then
    export QWEN_MODEL="qwen3-vl-flash"
  fi

  if [[ -z "${QWEN_BASE_URL:-}" ]]; then
    export QWEN_BASE_URL="${DASHSCOPE_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"
  fi
}
