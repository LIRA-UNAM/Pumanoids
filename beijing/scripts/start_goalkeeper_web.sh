#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."
DEMO_ROOT="$(pwd)"
WORKSPACE_ROOT="${DEMO_ROOT}/beijing_ws"
source "${WORKSPACE_ROOT}/install/setup.bash"

PORT="${GOALKEEPER_WEB_PORT:-8088}"
SOURCE_CONFIG="${WORKSPACE_ROOT}/src/brain/config/config_local.yaml"
INSTALLED_CONFIG="${WORKSPACE_ROOT}/install/share/brain/config/config_local.yaml"

pkill -f '[g]oalkeeper_web/server.py' 2>/dev/null || true
ARGS=(--host 0.0.0.0 --port "${PORT}" --config "${SOURCE_CONFIG}")
if [ -e "${INSTALLED_CONFIG}" ] && [ "${INSTALLED_CONFIG}" != "${SOURCE_CONFIG}" ]; then
  ARGS+=(--config "${INSTALLED_CONFIG}")
fi
nohup python3 "${WORKSPACE_ROOT}/src/brain/tools/goalkeeper_web/server.py" "${ARGS[@]}" \
  > "${DEMO_ROOT}/goalkeeper_web.log" 2>&1 &
echo "Goalkeeper web panel listening on port ${PORT}"
