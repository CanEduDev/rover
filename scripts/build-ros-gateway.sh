#!/usr/bin/env bash

set -eo pipefail

usage() {
    SCRIPT_NAME="$(basename "$0")"
    cat <<EOF

${SCRIPT_NAME} [-h | --help] [--push] [--no-cache] [--ros-distro DISTRO] [--platform PLATFORM] [--version-tags TAGS] [--package-name NAME] [--builder BUILDER]

Build the ros-gateway docker image. Requires docker to be installed
and usable without sudo.

Images are always loaded into the local docker daemon.
Use --push to also push to the registry.

args:
    --help              show this help
    --push              push containers to registry
    --no-cache          build without using docker cache
    --ros-distro DISTRO ROS distro codename to build (default: jazzy)
    --platform PLATFORM Target platform (default: native)
    --version-tags TAGS Space separated version tags
    --package-name NAME Docker container base name (default: ghcr.io/canedudev/rover/ros-gateway)
    --builder BUILDER   Docker builder to use (default: default)

Examples:
    # Build for native architecture
    ${SCRIPT_NAME} --ros-distro jazzy

    # Cross-compile for specific platform
    ${SCRIPT_NAME} --ros-distro jazzy --platform linux/arm64

    # Build with version tags and push
    ${SCRIPT_NAME} --ros-distro jazzy --version-tags "v1.0.0" --push

    # Build without cache
    ${SCRIPT_NAME} --ros-distro jazzy --no-cache

    # Build using specific builder
    ${SCRIPT_NAME} --ros-distro jazzy --builder my-builder
EOF

}

if ! command -v docker >/dev/null; then
    echo "error: docker executable not found"
    exit 1
fi

# Default values
ROS_DISTRO="jazzy"
PLATFORM=""
VERSION_TAGS=""
PACKAGE_NAME="ghcr.io/canedudev/rover/ros-gateway"
NO_CACHE=""
PUSH=""
BUILDER=""

while [[ $# -gt 0 ]]; do
    case "$1" in
    -h) ;&
    --help)
        usage
        exit 0
        ;;

    --push)
        PUSH="true"
        shift 1
        ;;

    --no-cache)
        NO_CACHE="true"
        shift 1
        ;;

    --ros-distro)
        ROS_DISTRO="$2"
        shift 2
        ;;

    --platform)
        PLATFORM="$2"
        shift 2
        ;;

    --version-tags)
        VERSION_TAGS="$2"
        shift 2
        ;;

    --package-name)
        PACKAGE_NAME="$2"
        shift 2
        ;;

    --builder)
        BUILDER="$2"
        shift 2
        ;;

    *)
        echo "error: unknown argument $1"
        usage
        exit 1
        ;;
    esac
done

ROOT_DIR=$(git rev-parse --show-toplevel)
BUILD_CONTEXT="${BUILD_CONTEXT:-${ROOT_DIR}}"
DOCKERFILE="${DOCKERFILE:-${ROOT_DIR}/ros2/Dockerfile}"

PACKAGE="${PACKAGE_NAME}-${ROS_DISTRO}"

# Set default build args
BUILD_ARGS=(
    "--build-arg" "ROS_DISTRO=${ROS_DISTRO}"
    "--provenance" "false"
    "--sbom" "false"
)

# If no version tags specified, use git describe
if [[ -z ${VERSION_TAGS} ]]; then
    VERSION_TAGS=$(git describe --tags --always)
fi

for tag in ${VERSION_TAGS}; do
    BUILD_ARGS+=("--tag" "${PACKAGE}:${tag}")
done

# CI caching
if [[ -n ${CI} ]]; then
    CACHE_ARCH="$(uname -m)"
    BUILD_ARGS+=(
        "--cache-from" "type=registry,ref=${PACKAGE}:buildcache-${CACHE_ARCH}"
        "--cache-to" "type=registry,ref=${PACKAGE}:buildcache-${CACHE_ARCH},mode=max"
    )
fi

# Build options
if [[ -n ${NO_CACHE} ]]; then
    BUILD_ARGS+=("--no-cache")
fi

if [[ -n ${PLATFORM} ]]; then
    BUILD_ARGS+=("--platform" "${PLATFORM}")
fi

# Add push if requested
if [[ -n ${PUSH} ]]; then
    BUILD_ARGS+=("--push")
elif [[ -z ${CI} ]]; then
    # Only load locally if not in CI
    BUILD_ARGS+=("--load")
    # Also add development tag
    BUILD_ARGS+=("--tag" "rover-ros-gateway:${ROS_DISTRO}")
fi

if [[ -n ${BUILDER} ]]; then
    BUILD_ARGS+=("--builder" "${BUILDER}")
fi

echo "Building ROS gateway for ${ROS_DISTRO}..."
if [[ -n ${PLATFORM} ]]; then
    echo "Target platform: ${PLATFORM}"
fi

docker build \
    -f "${DOCKERFILE}" \
    "${BUILD_ARGS[@]}" \
    "${BUILD_CONTEXT}"
