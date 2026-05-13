#!/usr/bin/env bash
# Builds isaac-lab-base → isaac-lab-leisaac → isaac-lab-groot before the devcontainer starts.
# Runs on the HOST via devcontainer initializeCommand.
# Pass --force to rebuild all images even if they already exist locally.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ENV_FILE="${REPO_ROOT}/docker/.env.base"

FORCE=0
for arg in "$@"; do [[ "$arg" == "--force" ]] && FORCE=1; done

build_if_needed() {
    local image="$1"; shift
    if [[ "$FORCE" -eq 0 ]] && docker image inspect "${image}" > /dev/null 2>&1; then
        echo "[devcontainer] ${image} already exists, skipping build. (pass --force to rebuild)"
        return
    fi
    echo "[devcontainer] Building ${image}..."
    docker build "$@"
}

# Load build args from .env.base
source "${ENV_FILE}"

build_if_needed isaac-lab-base \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.base" \
    --build-arg ISAACSIM_BASE_IMAGE_ARG="${ISAACSIM_BASE_IMAGE}" \
    --build-arg ISAACSIM_VERSION_ARG="${ISAACSIM_VERSION}" \
    --build-arg ISAACSIM_ROOT_PATH_ARG="${DOCKER_ISAACSIM_ROOT_PATH}" \
    --build-arg ISAACLAB_PATH_ARG="${DOCKER_ISAACLAB_PATH}" \
    --build-arg DOCKER_USER_HOME_ARG="${DOCKER_USER_HOME}" \
    -t isaac-lab-base \
    "${REPO_ROOT}"

build_if_needed isaac-lab-leisaac \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.leisaac" \
    -t isaac-lab-leisaac \
    "${REPO_ROOT}"

build_if_needed isaac-lab-groot \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.groot" \
    -t isaac-lab-groot \
    "${REPO_ROOT}"

echo "[devcontainer] Images ready."
