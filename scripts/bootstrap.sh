#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR=$(git rev-parse --show-toplevel)
TOOL_DIR=${ROOT_DIR}/.tools

TOOLCHAIN_URL=https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/15.3.rel1/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi.tar.xz
YAMLFMT_URL=https://github.com/google/yamlfmt/releases/download/v0.21.0/yamlfmt_0.21.0_Linux_x86_64.tar.gz

echo "Installing APT dependencies..."
sudo apt-get -qq update
sudo apt-get -qq install --no-upgrade -y \
    curl \
    openocd \
    zip

# Stamp the marker with the tool URLs so that bumping a version invalidates any
# previously installed toolchain instead of silently keeping the stale one.
TOOL_STAMP="${TOOLCHAIN_URL} ${YAMLFMT_URL}"

if [[ $(cat "${TOOL_DIR}/.complete" 2>/dev/null || true) != "${TOOL_STAMP}" ]]; then
    echo "Installing tools..."

    rm -rf "${TOOL_DIR}"
    mkdir -p "${TOOL_DIR}"
    pushd "${TOOL_DIR}" >/dev/null

    mkdir -p arm-gnu-toolchain
    curl --progress-bar -LSf "${TOOLCHAIN_URL}" | tar xJf - --strip-components=1 -C arm-gnu-toolchain
    curl --progress-bar -LSf "${YAMLFMT_URL}" | tar xzf - yamlfmt

    echo "${TOOL_STAMP}" >.complete
    popd >/dev/null
fi

if ! command -v uv; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

uv sync

# shellcheck disable=SC1091,SC2312
source "${ROOT_DIR}"/.venv/bin/activate

echo "Setting up build dir..."
meson setup --cross-file "${ROOT_DIR}"/stm32f302ret6.ini --wipe "${ROOT_DIR}"/build

echo "
Finished bootstrapping. Next steps:

1. Enter Python virtual environment using 'source .venv/bin/activate'
2. Build source using 'meson compile -C build'
"
