#!/bin/bash
set -e

echo "=== Booster Robot Setup ==="
echo "NOTE: this script requires an internet connection."
echo ""

# ---------------------------------------------------------------------------
# 1. Install the SDK (sdk_release/)
# ---------------------------------------------------------------------------
SDK_DIR="/home/booster/sdk_release"

if [ -d "$SDK_DIR" ]; then
    echo "--- Installing SDK from $SDK_DIR ---"
    cd "$SDK_DIR"

    echo "Fixing broken dependencies (apt --fix-broken install)..."
    sudo apt --fix-broken install -y

    echo "Running SDK install.sh..."
    sudo ./install.sh

    echo "Building SDK..."
    mkdir -p build
    cd build
    cmake ..
    make -j4

    echo "SDK installed successfully."
    echo ""
else
    echo "WARNING: folder $SDK_DIR not found. Skipping SDK installation."
    echo ""
fi

# ---------------------------------------------------------------------------
# 2. Install Pumanoids (beijing/scripts/install.sh)
# ---------------------------------------------------------------------------
SCRIPTS_DIR="/home/booster/Pumanoids/beijing/scripts"

if [ -d "$SCRIPTS_DIR" ]; then
    echo "--- Installing Pumanoids from $SCRIPTS_DIR ---"
    cd "$SCRIPTS_DIR/.."

    echo "Running scripts/install.sh..."
    sudo scripts/install.sh

    echo "Sourcing ROS kilted..."
    source /opt/ros/kilted/setup.bash

    echo "Pumanoids installed successfully."
    echo ""
else
    echo "WARNING: folder $SCRIPTS_DIR not found. Skipping Pumanoids installation."
    echo ""
fi
# ---------------------------------------------------------------------------
# 3. Add the aliases.sh source line to ~/.bashrc (expected at
#    <root>/Pumanoids/setup/booster_t2/aliases.sh)
# ---------------------------------------------------------------------------
ALIASES_FILE="$ROOT_DIR/Pumanoids/setup/booster_t2/aliases.sh"
 
if [ -f "$ALIASES_FILE" ]; then
    ALIASES_FILE="$(cd "$(dirname "$ALIASES_FILE")" && pwd)/$(basename "$ALIASES_FILE")"   # resolve to absolute path
 
    LINE="[ -f $ALIASES_FILE ] && source $ALIASES_FILE"
 
    echo "--- Registering aliases in ~/.bashrc ---"
    if grep -Fxq "$LINE" ~/.bashrc 2>/dev/null; then
        echo "Line already present in ~/.bashrc, skipping."
    else
        echo "" >> ~/.bashrc
        echo "# --- Booster robot aliases (added automatically) ---" >> ~/.bashrc
        echo "$LINE" >> ~/.bashrc
        echo "Line added to ~/.bashrc:"
        echo "  $LINE"
    fi
    echo ""
else
    echo "WARNING: expected file '$ALIASES_FILE' not found."
    echo "         'Pumanoids/setup/booster_t2/aliases.sh' must exist relative to the root of the machine ($ROOT_DIR)."
    echo "         Skipping aliases registration."
    echo ""
fi
 
echo "=== Setup complete ==="
echo "Run 'source ~/.bashrc' or open a new terminal to see the changes."