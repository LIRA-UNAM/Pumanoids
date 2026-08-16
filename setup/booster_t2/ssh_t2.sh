#!/bin/bash

# 1. Crear la carpeta .ssh con los permisos correctos si no existe
mkdir -p ~/.ssh
chmod 700 ~/.ssh

# 2. Archivo de configuración
CONFIG_FILE="$HOME/.ssh/config"

# 3. Agregar los alias al final del archivo sin tocar lo existente (usando >>)
cat <<EOF >> "$CONFIG_FILE"

Host wifi_4
  HostName 192.168.1.194
  User booster

Host wifi_3
  HostName 192.168.1.193
  User booster

Host wifi_5
  HostName 192.168.1.195
  User booster
EOF

# 4. Asegurar los permisos correctos
chmod 600 "$CONFIG_FILE"

echo "To connect to the robots ssh and one of the following users: | wifi_3 | wifi_4 | wifi_5 |"
