#!/bin/bash
set -e

echo "=== Wifi static IP configuration ==="

# --- Datos de la red WiFi ---
read -p "SSID of wifi network: " SSID
read -s -p "Password: " WIFI_PASS
echo ""

# --- Datos de IP estática ---
read -p "IP to set (ex: 192.168.1.50/24): " IP_CIDR
read -p "Gateway (void if not applicable): " GATEWAY
read -p "DNS (void to set: 8.8.8.8): " DNS
DNS=${DNS:-8.8.8.8}

CON_NAME="$SSID"

# --- Verificar si ya existe una conexión con ese nombre ---
if nmcli connection show "$CON_NAME" &> /dev/null; then
    echo "Ya existe una conexión llamada '$CON_NAME'. Se va a modificar."
else
    echo "Creando nueva conexión WiFi '$CON_NAME'..."
    sudo nmcli connection add type wifi con-name "$CON_NAME" ifname wlan0 ssid "$SSID"
fi

# --- Configurar seguridad WiFi ---
sudo nmcli connection modify "$CON_NAME" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$WIFI_PASS"

# --- Configurar IP estática ---
if [ -n "$GATEWAY" ]; then
    sudo nmcli connection modify "$CON_NAME" \
        ipv4.addresses "$IP_CIDR" \
        ipv4.gateway "$GATEWAY" \
        ipv4.dns "$DNS" \
        ipv4.method manual
else
    sudo nmcli connection modify "$CON_NAME" \
        ipv4.addresses "$IP_CIDR" \
        ipv4.dns "$DNS" \
        ipv4.method manual
fi

# --- Activar la conexión ---
echo "Activating conection..."
sudo nmcli connection up "$CON_NAME"

echo ""
echo "=== IP's set succesfully! ==="
nmcli connection show "$CON_NAME" | grep -E "ipv4|wifi"
