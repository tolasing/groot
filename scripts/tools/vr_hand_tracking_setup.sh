#!/usr/bin/env bash
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# VR Hand Tracking Setup Script
#
# Starts the CloudXR runtime and the Isaac Lab VR demo recording session
# for the G1 locomanipulation pick-and-place task using Meta Quest optical
# hand tracking (no physical controllers required).
#
# Prerequisites:
#   - Isaac Sim 6.0.0-dev2 container with isaacteleop / CloudXR extensions
#   - Meta Quest headset connected and CloudXR client app launched
#   - CloudXR signaling server reachable on port 49100
#
# Usage:
#   # Terminal 1 — start CloudXR runtime (keep alive, do not kill)
#   bash scripts/tools/vr_hand_tracking_setup.sh cloudxr
#
#   # Terminal 2 — start the recording session (after CloudXR prints "runtime ready")
#   bash scripts/tools/vr_hand_tracking_setup.sh record [--num_demos N] [--dataset_file PATH]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ---------------------------------------------------------------------------
# CloudXR environment — NV_CXR_ENABLE_PUSH_DEVICES=0 MUST be set before
# launching isaacteleop.cloudxr so that EnvConfig picks it up from the
# process environment and disables Manus glove push-device mode, enabling
# Meta Quest built-in optical hand tracking instead.
# ---------------------------------------------------------------------------
export NV_CXR_ENABLE_PUSH_DEVICES=0
export NV_CXR_ENABLE_TENSOR_DATA=true
export NV_CXR_FILE_LOGGING=true
export NV_DEVICE_PROFILE=auto-webrtc

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
usage() {
    echo "Usage: $0 {cloudxr|record} [extra args for record_demos.py]"
    echo ""
    echo "  cloudxr   Start the CloudXR OpenXR runtime (keep this terminal open)"
    echo "  record    Launch the VR demo recording session"
    exit 1
}

start_cloudxr() {
    echo "[vr_setup] Starting CloudXR runtime with optical hand tracking..."
    echo "[vr_setup] NV_CXR_ENABLE_PUSH_DEVICES=$NV_CXR_ENABLE_PUSH_DEVICES (0 = optical hand tracking)"
    echo ""
    cd "$REPO_ROOT"
    exec ./isaaclab.sh -p -m isaacteleop.cloudxr --accept-eula
}

start_record() {
    local extra_args=("$@")

    # Source the CloudXR env file written by the runtime (if it exists)
    if [[ -f /root/.cloudxr/run/cloudxr.env ]]; then
        # shellcheck disable=SC1091
        source /root/.cloudxr/run/cloudxr.env
        # Re-apply hand tracking flag (cloudxr.env may overwrite it)
        export NV_CXR_ENABLE_PUSH_DEVICES=0
    fi

    echo "[vr_setup] Launching Isaac Lab VR recording session..."
    cd "$REPO_ROOT"
    exec ./isaaclab.sh -p scripts/tools/record_demos.py \
        --task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
        --xr --headless \
        "${extra_args[@]}"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
CMD="${1:-}"
shift || true

case "$CMD" in
    cloudxr) start_cloudxr ;;
    record)  start_record "$@" ;;
    *)       usage ;;
esac
