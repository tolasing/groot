#!/usr/bin/env bash
# Builds isaac-lab-base before the devcontainer starts.
# Runs on the HOST via devcontainer initializeCommand.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ENV_FILE="${REPO_ROOT}/docker/.env.base"

source "${ENV_FILE}"

echo "[devcontainer] Building isaac-lab-base..."
docker build \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.base" \
    --build-arg ISAACSIM_BASE_IMAGE_ARG="${ISAACSIM_BASE_IMAGE}" \
    --build-arg ISAACSIM_VERSION_ARG="${ISAACSIM_VERSION}" \
    --build-arg ISAACSIM_ROOT_PATH_ARG="${DOCKER_ISAACSIM_ROOT_PATH}" \
    --build-arg ISAACLAB_PATH_ARG="${DOCKER_ISAACLAB_PATH}" \
    --build-arg DOCKER_USER_HOME_ARG="${DOCKER_USER_HOME}" \
    -t isaac-lab-base \
    "${REPO_ROOT}"

echo "[devcontainer] Image ready."
