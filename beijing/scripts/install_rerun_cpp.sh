#!/usr/bin/env bash

set -euo pipefail

RERUN_VERSION="0.20.3"
ARROW_VERSION="18.0.0"
INSTALL_PREFIX="${RERUN_SDK_PREFIX:-/opt/rerun_sdk}"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/robocup"
DEFAULT_ARCHIVE="$CACHE_DIR/rerun_cpp_sdk-${RERUN_VERSION}-multiplatform.zip"
URL="https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk-${RERUN_VERSION}-multiplatform.zip"
WORK_DIR="$(mktemp -d)"

CONFIG_FILE="$INSTALL_PREFIX/lib/cmake/rerun_sdk/rerun_sdkConfig.cmake"
VERSION_FILE="$INSTALL_PREFIX/lib/cmake/rerun_sdk/rerun_sdkConfigVersion.cmake"

if [[ "${RERUN_FORCE_REINSTALL:-0}" != "1" ]] &&
   [[ -f "$CONFIG_FILE" ]] && [[ -f "$VERSION_FILE" ]] &&
   grep -q "set(PACKAGE_VERSION \"${RERUN_VERSION}\")" "$VERSION_FILE"; then
    echo "Rerun C++ SDK ${RERUN_VERSION} already installed in ${INSTALL_PREFIX}; skipping build."
    exit 0
fi

if [[ $# -gt 2 ]]; then
    echo "Usage: $0 [rerun_cpp_sdk-${RERUN_VERSION}-multiplatform.zip] [apache-arrow-${ARROW_VERSION}.tar.gz]" >&2
    exit 2
fi

ARCHIVE="${1:-$DEFAULT_ARCHIVE}"
ARROW_ARCHIVE="${2:-}"

cleanup() {
    rm -rf "$WORK_DIR"
}
trap cleanup EXIT

mkdir -p "$CACHE_DIR"

download_sdk() {
    curl --fail --location \
        --retry 5 --retry-delay 2 --retry-all-errors \
        --continue-at - \
        --output "$ARCHIVE" \
        "$URL"
}

if [[ $# -ge 1 ]]; then
    if [[ ! -f "$ARCHIVE" ]]; then
        echo "Rerun SDK archive not found: $ARCHIVE" >&2
        exit 1
    fi
    if ! unzip -tq "$ARCHIVE" >/dev/null 2>&1; then
        echo "Invalid or incomplete Rerun SDK archive: $ARCHIVE" >&2
        exit 1
    fi
else
    if [[ ! -f "$ARCHIVE" ]] || ! unzip -tq "$ARCHIVE" >/dev/null 2>&1; then
        if [[ -f "$ARCHIVE" ]]; then
            echo "Resuming the incomplete Rerun SDK download." >&2
        fi
        download_sdk
    fi

    if ! unzip -tq "$ARCHIVE" >/dev/null 2>&1; then
        echo "Rerun SDK archive is still incomplete; downloading it from scratch." >&2
        rm -f "$ARCHIVE"
        download_sdk
        unzip -tq "$ARCHIVE" >/dev/null
    fi
fi

unzip -q "$ARCHIVE" -d "$WORK_DIR"
SOURCE_DIR="$WORK_DIR/rerun_cpp_sdk"
BUILD_DIR="$WORK_DIR/build"

if [[ -n "$ARROW_ARCHIVE" ]]; then
    if [[ ! -f "$ARROW_ARCHIVE" ]]; then
        echo "Apache Arrow archive not found: $ARROW_ARCHIVE" >&2
        exit 1
    fi
    if ! tar -tzf "$ARROW_ARCHIVE" >/dev/null 2>&1; then
        echo "Invalid or incomplete Apache Arrow archive: $ARROW_ARCHIVE" >&2
        exit 1
    fi

    LOCAL_ARROW_ARCHIVE="$WORK_DIR/apache-arrow-${ARROW_VERSION}.tar.gz"
    cp "$ARROW_ARCHIVE" "$LOCAL_ARROW_ARCHIVE"
    sed -i \
        "/^[[:space:]]*GIT_REPOSITORY /,/^[[:space:]]*GIT_PROGRESS /c\\
        URL \"file://${LOCAL_ARROW_ARCHIVE}\"\\
        DOWNLOAD_EXTRACT_TIMESTAMP ON" \
        "$SOURCE_DIR/download_and_build_arrow.cmake"
fi

cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DBUILD_SHARED_LIBS=OFF \
    -DRERUN_DOWNLOAD_AND_BUILD_ARROW=ON
cmake --build "$BUILD_DIR" --parallel "$(nproc)"
if (( EUID == 0 )); then
    INSTALL_COMMAND=()
else
    INSTALL_COMMAND=(sudo)
fi
"${INSTALL_COMMAND[@]}" cmake --install "$BUILD_DIR"

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Rerun SDK installation failed: $CONFIG_FILE was not created." >&2
    exit 1
fi

echo "Rerun C++ SDK ${RERUN_VERSION} installed in ${INSTALL_PREFIX}."
