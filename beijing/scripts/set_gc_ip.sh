#!/bin/bash
#
# set_game_controller_ip.sh
#
# Sobreescribe la IP del GameController en:
#   - .../brain/config/config.yaml            (clave: game_control_ip)
#   - .../game_controller/launch/launch.py     (default_value del argumento ip_white_list)
#
# Uso:
#   ./set_game_controller_ip.sh ip:=x.x.x.x
#
# Ejemplo:
#   ./set_game_controller_ip.sh ip:=192.168.10.100
#
set -euo pipefail

# --- Rutas de los archivos a modificar ---
CONFIG_YAML="/home/booster/Pumanoids/beijing/beijing_ws/src/brain/config/config.yaml"
LAUNCH_PY="/home/booster/Pumanoids/beijing/beijing_ws/src/game_controller/launch/launch.py"

usage() {
    echo "Use: $0 ip:=x.x.x.x"
    echo "Ex: $0 ip:=192.168.10.100"
    exit 1
}

if [ $# -lt 1 ]; then
    usage
fi

ARG="$1"

# Aceptar tanto "ip:=x.x.x.x" como una IP suelta "x.x.x.x"
if [[ "$ARG" == ip:=* ]]; then
    NEW_IP="${ARG#ip:=}"
else
    NEW_IP="$ARG"
fi

# --- Validar formato de IPv4 ---
IP_REGEX='^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$'
if [[ ! "$NEW_IP" =~ $IP_REGEX ]]; then
    echo "Error: '$NEW_IP' no tiene formato de IP valido (esperado x.x.x.x)"
    usage
fi

for octet in ${NEW_IP//./ }; do
    if [ "$octet" -gt 255 ]; then
        echo "Error: '$NEW_IP' no es una IP valida (octeto '$octet' fuera de rango 0-255)"
        exit 1
    fi
done

echo "=> Nueva IP del GameController: $NEW_IP"
echo

# ------------------------------------------------------------------
# 1) config.yaml -> game_control_ip: "x.x.x.x"
# ------------------------------------------------------------------
if [ ! -f "$CONFIG_YAML" ]; then
    echo "Error: no se encontro $CONFIG_YAML"
    exit 1
fi

cp "$CONFIG_YAML" "${CONFIG_YAML}.bak"

sed -i -E \
    "s/(game_control_ip:[[:space:]]*\")[0-9]{1,3}(\.[0-9]{1,3}){3}(\")/\1${NEW_IP}\3/" \
    "$CONFIG_YAML"

if grep -q "game_control_ip:[[:space:]]*\"${NEW_IP}\"" "$CONFIG_YAML"; then
    echo "OK  config.yaml actualizado -> game_control_ip: \"${NEW_IP}\""
else
    echo "AVISO: no se pudo confirmar el cambio en config.yaml."
    echo "       Revisa manualmente la clave 'game_control_ip'."
fi

# ------------------------------------------------------------------
# 2) launch.py -> DeclareLaunchArgument('ip_white_list', default_value='x.x.x.x', ...)
# ------------------------------------------------------------------
if [ ! -f "$LAUNCH_PY" ]; then
    echo "Error: no se encontro $LAUNCH_PY"
    exit 1
fi

cp "$LAUNCH_PY" "${LAUNCH_PY}.bak"

# Solo toca la linea que declara 'ip_white_list' junto con su default_value
sed -i -E \
    "/'ip_white_list'.*default_value/ s/default_value='[0-9]{1,3}(\.[0-9]{1,3}){3}(,[0-9]{1,3}(\.[0-9]{1,3}){3})*'/default_value='${NEW_IP}'/" \
    "$LAUNCH_PY"

if grep -q "'ip_white_list', default_value='${NEW_IP}'" "$LAUNCH_PY"; then
    echo "OK  launch.py actualizado -> ip_white_list default_value='${NEW_IP}'"
else
    echo "AVISO: no se pudo confirmar el cambio en launch.py."
    echo "       Revisa manualmente el argumento 'ip_white_list'."
fi

echo
echo "Backups creados:"
echo "  ${CONFIG_YAML}.bak"
echo "  ${LAUNCH_PY}.bak"
