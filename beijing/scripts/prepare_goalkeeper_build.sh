#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."
DEMO_ROOT="$(pwd)"
STAGING_ROOT="${DEMO_ROOT}/beijing_ws"
cd "${STAGING_ROOT}"

ROS_SETUP=""
for candidate in /opt/ros/kilted/setup.bash /opt/ros/humble/setup.bash; do
  if [ -f "${candidate}" ]; then
    ROS_SETUP="${candidate}"
    break
  fi
done
if [ -z "${ROS_SETUP}" ]; then
  echo "ERROR: no se encontró ROS 2 Kilted ni Humble en /opt/ros" >&2
  exit 1
fi
if [ ! -f /usr/local/include/booster/robot/b1/b1_loco_api.hpp ]; then
  echo "ERROR: no se encontró la SDK pública de Booster en /usr/local/include" >&2
  exit 1
fi

# Los setup scripts de ROS consultan variables opcionales que pueden no existir.
# Desactive nounset solo mientras se cargan y restaurelo inmediatamente despues.
set +u
source "${ROS_SETUP}"
if [ -f "${DEMO_ROOT}/install/setup.bash" ]; then
  source "${DEMO_ROOT}/install/setup.bash"
fi
set -u

python3 -m py_compile \
  src/brain/tools/goalkeeper_web/server.py \
  src/brain/test/goalkeeper_web_test.py
python3 src/brain/test/goalkeeper_web_test.py \
  --server-dir src/brain/tools/goalkeeper_web

python3 - <<'PY'
import ast
import re
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path.cwd()
server = root / "src/brain/tools/goalkeeper_web/server.py"
brain = root / "src/brain/src/brain.cpp"
tree = root / "src/brain/src/brain_tree.cpp"
ast.parse(server.read_text(encoding="utf-8"), filename=str(server))
ET.parse(root / "src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml")
schema = set(re.findall(
    r'^\s*"([A-Za-z][\w.]+)":\s*(?:number|integer|boolean|choice)\(',
    server.read_text(encoding="utf-8"), re.M))
cpp = brain.read_text(encoding="utf-8")
declared = set(re.findall(
    r'declare_parameter(?:<[^>]+>)?\(\s*"([^"]+)"', cpp))
access = set(re.findall(
    r'get_parameter\(\s*"(goalkeeper\.[^"]+)"',
    cpp + tree.read_text(encoding="utf-8")))
assert schema <= declared, f"GUI no declarada: {sorted(schema - declared)}"
assert access <= declared, f"Acceso no declarado: {sorted(access - declared)}"
print(f"Preflight estático OK: {len(schema)} parámetros web")
PY

chmod +x "${DEMO_ROOT}/scripts/start_goalkeeper_web.sh" \
  "${DEMO_ROOT}/scripts/prepare_goalkeeper_build.sh"

echo "[BUILD brain]"
cd "${DEMO_ROOT}"
# El repositorio conserva los fuentes en beijing_ws/src como staging, pero los
# scripts originales ejecutan el release plano desde beijing/install.
colcon build \
  --base-paths "${STAGING_ROOT}/src" \
  --build-base "${DEMO_ROOT}/build" \
  --install-base "${DEMO_ROOT}/install" \
  --packages-select brain \
  --cmake-clean-cache \
  --event-handlers console_direct+

set +u
source "${DEMO_ROOT}/install/setup.bash"
set -u
echo "[TEST brain: pruebas funcionales]"
# El demo original acumula deuda en los linters globales (especialmente
# uncrustify). Para decidir si el robot esta listo, ejecute los tests de codigo,
# integracion web y compatibilidad con la SDK; los linters se revisan aparte.
ctest --test-dir "${DEMO_ROOT}/build/brain" \
  -E '^(cppcheck|flake8|lint_cmake|pep257|uncrustify|xmllint)$' \
  --output-on-failure

echo "[CHECK runtime files]"
test -f "${DEMO_ROOT}/install/brain/share/brain/tools/goalkeeper_web/server.py"
test -f "${DEMO_ROOT}/install/brain/share/brain/config/config_local.yaml"
test -f "${DEMO_ROOT}/install/brain/share/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml"
grep -aFq 'goalkeeper.prediction.reject_outside_field' \
  "${DEMO_ROOT}/install/brain/lib/brain/brain_node"
grep -Fq 'source ./install/setup.bash' "${DEMO_ROOT}/scripts/start.sh"

echo "PREPARACION OK. Inicie con: cd ${DEMO_ROOT} && ./scripts/start.sh role:=goal_keeper"
echo "Panel: http://IP_DEL_ROBOT:8088"
