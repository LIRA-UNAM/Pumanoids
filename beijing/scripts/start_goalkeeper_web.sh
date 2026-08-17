#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."
DEMO_ROOT="$(pwd)"
PORT="${GOALKEEPER_WEB_PORT:-8088}"
SOURCE_ROOT="${DEMO_ROOT}/beijing_ws"
SOURCE_CONFIG="${SOURCE_ROOT}/src/brain/config/config_local.yaml"
INSTALLED_CONFIG="${DEMO_ROOT}/install/brain/share/brain/config/config_local.yaml"
LOG_DIR="${DEMO_ROOT}/goalkeeper_logs"

# rclpy intercepta SIGTERM y puede dejar vivo el servidor HTTP con el contexto
# ROS apagado. Termine por completo la instancia anterior antes de reutilizar
# el puerto 8088.
pkill -9 -f '[g]oalkeeper_web/server.py' 2>/dev/null || true
ARGS=(--host 0.0.0.0 --port "${PORT}" --log-dir "${LOG_DIR}" --config "${SOURCE_CONFIG}")
if [ -e "${INSTALLED_CONFIG}" ] && [ "${INSTALLED_CONFIG}" != "${SOURCE_CONFIG}" ]; then
  ARGS+=(--config "${INSTALLED_CONFIG}")
fi
nohup setsid python3 "${SOURCE_ROOT}/src/brain/tools/goalkeeper_web/server.py" "${ARGS[@]}" \
  </dev/null \
  > "${DEMO_ROOT}/goalkeeper_web.log" 2>&1 &
echo "Goalkeeper web panel listening on port ${PORT}"
echo "Telemetry logs: ${LOG_DIR}"
