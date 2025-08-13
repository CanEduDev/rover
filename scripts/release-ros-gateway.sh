#!/usr/bin/env bash

set -eo pipefail

usage() {
    SCRIPT_NAME=$(basename "$0")
    cat <<EOF

${SCRIPT_NAME} [-h | --help] [--version VERSION] [--package-name NAME]

Create multi-platform Docker manifests for the ros-gateway images.
Requires docker to be installed and authenticated to the registry.

args:
    --help                  show this help
    --version VERSION       version tag to release
    --package-name NAME     Docker container base name (default: ghcr.io/canedudev/rover/ros-gateway)

Examples:
    # Create manifests for latest
    ${SCRIPT_NAME}

    # Create manifests for specific version
    ${SCRIPT_NAME} --version v1.0.0

    # Create manifests for custom package name
    ${SCRIPT_NAME} --package-name "my-registry/my-project/ros-gateway"
EOF
}

# Default values
VERSION=$(git describe --tags --always)
PACKAGE_NAME="ghcr.io/canedudev/rover/ros-gateway"

while [[ $# -gt 0 ]]; do
    case "$1" in
    -h) ;&
    --help)
        usage
        exit 0
        ;;

    --package-name)
        PACKAGE_NAME="$2"
        shift 2
        ;;

    --version)
        VERSION="$2"
        shift 2
        ;;

    *)
        echo "error: unknown argument $1"
        usage
        exit 1
        ;;
    esac
done

# ROS distros to create manifests for
ROS_DISTROS=("humble" "jazzy")

echo "Creating release package for ${VERSION}..."

for distro in "${ROS_DISTROS[@]}"; do
    PACKAGE="${PACKAGE_NAME}-${distro}"

    # Create version-specific manifest
    echo "Creating multi-platform manifest for ${distro}..."

    docker manifest create "${PACKAGE}:${VERSION}" \
        "${PACKAGE}:${VERSION}-x86_64" \
        "${PACKAGE}:${VERSION}-aarch64"

    docker manifest push "${PACKAGE}:${VERSION}"
done

echo "Multi-platform manifests created successfully!"
