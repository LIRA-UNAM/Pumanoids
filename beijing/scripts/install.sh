#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/kilted/setup.bash"
ROS_DISTRO_NAME="kilted"
RERUN_ARCHIVE="$REPO_ROOT/third_party/rerun_cpp_sdk-0.20.3-multiplatform.zip"
ARROW_ARCHIVE="$REPO_ROOT/third_party/apache-arrow-18.0.0.tar.gz"
RERUN_PREFIX="${RERUN_SDK_PREFIX:-/opt/rerun_sdk}"

# The default mirror is reachable from networks where raw.githubusercontent.com
# is blocked. Override it with ROSDEP_MIRROR when another mirror is preferred.
ROSDEP_MIRROR="${ROSDEP_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn}"
ROSDEP_MIRROR="${ROSDEP_MIRROR%/}"
ROSDEP_INDEX_URL="${ROSDISTRO_INDEX_URL:-${ROSDEP_MIRROR}/rosdistro/index-v4.yaml}"
ROSDEP_SOURCES_URL="${ROSDEP_SOURCES_URL:-${ROSDEP_MIRROR}/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS 2 Kilted was not found: $ROS_SETUP" >&2
    echo "Install ROS 2 Kilted before running this installer." >&2
    exit 1
fi

if (( EUID == 0 )); then
    SUDO=()
else
    if ! command -v sudo >/dev/null 2>&1; then
        echo "sudo is required when this installer is not run as root." >&2
        exit 1
    fi
    SUDO=(sudo)
fi

# ROS setup scripts from different ROS 2 releases can reference optional
# environment variables before defining them. Keep the installer strict while
# allowing the distro setup file to initialize its own environment.
set +u
source "$ROS_SETUP"
set -u

echo "[1/4] Checking system and ROS dependencies"
REQUIRED_PACKAGES=(
    build-essential \
    cmake \
    curl \
    git \
    ninja-build \
    unzip \
    ros-kilted-behaviortree-cpp \
    ros-kilted-backward-ros \
    libceres-dev \
    libfftw3-dev \
    espeak
)
MISSING_PACKAGES=()
APT_UPDATED=0
for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! dpkg-query -W -f='${db:Status-Abbrev}' "$package" 2>/dev/null | grep -q '^ii '; then
        MISSING_PACKAGES+=("$package")
    fi
done
if [[ "${ROSDEP_SKIP:-0}" != "1" ]] &&
   ! command -v rosdep >/dev/null 2>&1; then
    MISSING_PACKAGES+=(python3-rosdep)
fi
if ((${#MISSING_PACKAGES[@]} > 0)); then
    echo "Installing missing packages: ${MISSING_PACKAGES[*]}"
    "${SUDO[@]}" apt-get update
    APT_UPDATED=1
    "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y "${MISSING_PACKAGES[@]}"
else
    echo "All required system and ROS packages are already installed."
fi

echo "[2/4] Checking Rerun C++ SDK"
# Configure the repository filters without touching an existing pre-push hook.
# Some deployments keep a custom Gitee proxy-aware LFS hook in place.
RERUN_CONFIG_FILE="$RERUN_PREFIX/lib/cmake/rerun_sdk/rerun_sdkConfig.cmake"
RERUN_VERSION_FILE="$RERUN_PREFIX/lib/cmake/rerun_sdk/rerun_sdkConfigVersion.cmake"
if [[ "${RERUN_FORCE_REINSTALL:-0}" != "1" ]] &&
   [[ -f "$RERUN_CONFIG_FILE" ]] && [[ -f "$RERUN_VERSION_FILE" ]] &&
   grep -q 'set(PACKAGE_VERSION "0.20.3")' "$RERUN_VERSION_FILE"; then
    echo "Rerun C++ SDK 0.20.3 already installed in $RERUN_PREFIX; skipping build."
else
    NEEDS_LFS=0
    for archive in "$RERUN_ARCHIVE" "$ARROW_ARCHIVE"; do
        if [[ ! -f "$archive" ]] ||
           head -c 128 "$archive" 2>/dev/null | grep -aq \
               '^version https://git-lfs.github.com/spec/v1$'; then
            NEEDS_LFS=1
        fi
    done
    if ((NEEDS_LFS)); then
        if ! command -v git-lfs >/dev/null 2>&1; then
            if ((APT_UPDATED == 0)); then
                "${SUDO[@]}" apt-get update
                APT_UPDATED=1
            fi
            "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y git-lfs
        fi
        # Configure repository filters without touching an existing pre-push hook.
        git -C "$REPO_ROOT" lfs install --local --skip-repo
        for archive in "$RERUN_ARCHIVE" "$ARROW_ARCHIVE"; do
            if [[ ! -f "$archive" ]] ||
               head -c 128 "$archive" 2>/dev/null | grep -aq \
                   '^version https://git-lfs.github.com/spec/v1$'; then
                git -C "$REPO_ROOT" lfs pull --include="$(basename "$archive")"
            fi
        done
    fi

    if [[ ! -f "$RERUN_ARCHIVE" ]] ||
       ! unzip -tq "$RERUN_ARCHIVE" >/dev/null 2>&1; then
        echo "Invalid Rerun SDK archive: $RERUN_ARCHIVE" >&2
        echo "Run 'git lfs pull' and retry." >&2
        exit 1
    fi
    if [[ ! -f "$ARROW_ARCHIVE" ]] ||
       ! tar -tzf "$ARROW_ARCHIVE" >/dev/null 2>&1; then
        echo "Invalid Apache Arrow archive: $ARROW_ARCHIVE" >&2
        echo "Run 'git lfs pull' and retry." >&2
        exit 1
    fi

    RERUN_SDK_PREFIX="$RERUN_PREFIX" bash "$SCRIPT_DIR/install_rerun_cpp.sh" \
        "$RERUN_ARCHIVE" \
        "$ARROW_ARCHIVE"
fi

echo "[3/4] Updating rosdep"
if [[ "${ROSDEP_SKIP:-0}" == "1" ]]; then
    echo "ROSDEP_SKIP=1: skipping rosdep initialization and dependency installation."
else
    ROSDEP_LIST_DIR="/etc/ros/rosdep/sources.list.d"
    ROSDEP_LIST_FILE="$ROSDEP_LIST_DIR/20-default.list"
    if [[ ! -s "$ROSDEP_LIST_FILE" ]]; then
        echo "rosdep sources are not initialized; downloading ${ROSDEP_SOURCES_URL}"
        ROSDEP_TMP="$(mktemp)"
        trap 'rm -f "$ROSDEP_TMP"' EXIT
        if ! curl --fail --location --noproxy '*' --output "$ROSDEP_TMP" "$ROSDEP_SOURCES_URL"; then
            echo "Unable to download rosdep sources. Check network or set ROSDEP_SKIP=1." >&2
            exit 1
        fi
        "${SUDO[@]}" install -D -m 0644 "$ROSDEP_TMP" "$ROSDEP_LIST_FILE"
        rm -f "$ROSDEP_TMP"
        trap - EXIT
    else
        echo "rosdep sources already initialized; skipping rosdep init."
    fi

    # rosdep versions from ROS 1 used legacy gbpdistro entries in these files.
    # They are obsolete for ROS 2 and can make `rosdep update` fail even when
    # the requested ROS 2 distribution is available. Remove only those stale
    # entries; leave regular and project-specific YAML sources untouched.
    for rosdep_source_file in "$ROSDEP_LIST_DIR"/*.list; do
        [[ -f "$rosdep_source_file" ]] || continue
        if grep -Eq '^[[:space:]]*(gbpdistro|yaml)[[:space:]].*/releases/(electric|fuerte)\.yaml([[:space:]]|$)' \
            "$rosdep_source_file"; then
            ROSDEP_TMP="$(mktemp)"
            sed -E '/^[[:space:]]*(gbpdistro|yaml)[[:space:]].*\/releases\/(electric|fuerte)\.yaml([[:space:]]|$)/d' \
                "$rosdep_source_file" > "$ROSDEP_TMP"
            "${SUDO[@]}" install -D -m 0644 "$ROSDEP_TMP" "$rosdep_source_file"
            rm -f "$ROSDEP_TMP"
            echo "Removed obsolete electric/fuerte rosdep sources from $rosdep_source_file."
        fi
    done

    # Replace only the stock default source list when it still points to
    # GitHub. Keep any additional project-specific source files untouched.
    if grep -q 'raw.githubusercontent.com/ros/rosdistro' "$ROSDEP_LIST_FILE"; then
        ROSDEP_TMP="$(mktemp)"
        sed "s#https://raw.githubusercontent.com/ros/rosdistro#${ROSDEP_MIRROR}/github-raw/ros/rosdistro#g" \
            "$ROSDEP_LIST_FILE" > "$ROSDEP_TMP"
        "${SUDO[@]}" install -D -m 0644 "$ROSDEP_TMP" "$ROSDEP_LIST_FILE"
        rm -f "$ROSDEP_TMP"
        echo "Updated the default rosdep source list to ${ROSDEP_MIRROR}."
    fi

    if ! env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
        -u http_proxy -u https_proxy -u all_proxy \
        ROSDISTRO_INDEX_URL="$ROSDEP_INDEX_URL" \
        rosdep update --rosdistro "$ROS_DISTRO_NAME"; then
        echo "rosdep update failed while downloading one or more sources or the distribution index." >&2
        echo "Distribution index: ${ROSDEP_INDEX_URL}" >&2
        echo "Fix network/proxy and rerun, or use ROSDEP_SKIP=1 if dependencies are already installed." >&2
        exit 1
    fi

    echo "[4/4] Installing workspace dependencies"
    rosdep install \
        --from-paths "$REPO_ROOT/src" \
        --ignore-src \
        -r \
        -y \
        --rosdistro "$ROS_DISTRO_NAME"
fi

echo "Installation complete."
echo "For a new terminal, run: source $ROS_SETUP"
