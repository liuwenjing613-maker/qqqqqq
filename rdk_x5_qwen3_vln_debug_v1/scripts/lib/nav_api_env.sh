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

# Force-apply only the three visual-nav API exports from a file.
# Later calls win. Shell exports are overridden when the file has a real value.
_force_nav_api_exports_from_file() {
  local env_file="$1"
  local line key value
  [[ -f "$env_file" ]] || return 0

  while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"
    [[ -z "$line" || "$line" == \#* ]] && continue
    [[ "$line" == export\ * ]] && line="${line#export }"
    case "$line" in
      DASHSCOPE_API_KEY=*|QWEN_BASE_URL=*|QWEN_MODEL=*|DASHSCOPE_BASE_URL=*)
        key="${line%%=*}"
        value="${line#*=}"
        value="${value#\"}"
        value="${value%\"}"
        value="${value#\'}"
        value="${value%\'}"
        if [[ "$key" == "DASHSCOPE_BASE_URL" ]]; then
          # Alias only fills QWEN_BASE_URL when that key is absent in this pass.
          if [[ -n "$value" ]] && _nav_api_is_placeholder "${QWEN_BASE_URL:-}"; then
            export QWEN_BASE_URL="$value"
          fi
          continue
        fi
        if ! _nav_api_is_placeholder "$value"; then
          export "$key=$value"
        fi
        ;;
    esac
  done < "$env_file"
}

# Load visual-navigation API settings.
# Priority (highest last / wins):
#   1) existing shell env (temporary)
#   2) $project_dir/.env
#   3) $nav_root/.env.local
#   4) optional override file (e.g. voice_interaction/.env) — highest
# Args: project_dir nav_root [override_env_file]
load_nav_api_env() {
  local project_dir="$1"
  local nav_root="$2"
  local override_env_file="${3:-}"
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

  # Re-apply the three API exports from configs so they beat any prior shell value.
  _force_nav_api_exports_from_file "$project_dir/.env"
  _force_nav_api_exports_from_file "$nav_root/.env.local"
  if [[ -n "$override_env_file" ]]; then
    _force_nav_api_exports_from_file "$override_env_file"
  fi

  if _nav_api_is_placeholder "${QWEN_MODEL:-}"; then
    export QWEN_MODEL="qwen3-vl-flash"
  fi

  if _nav_api_is_placeholder "${QWEN_BASE_URL:-}"; then
    export QWEN_BASE_URL="${DASHSCOPE_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"
  fi
}
