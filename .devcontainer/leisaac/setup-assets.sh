#!/usr/bin/env bash
set -e

RELEASE="https://github.com/LightwheelAI/leisaac/releases/download/v0.1.0"
LEISAAC="/workspace/leisaac"

mkdir -p "${LEISAAC}/assets/robot"
mkdir -p "${LEISAAC}/assets/scenes"

# Robot USD
if [ ! -f "${LEISAAC}/assets/robot/so101_follower.usd" ]; then
    echo "Downloading so101_follower.usd..."
    curl -L "${RELEASE}/so101_follower.usd" -o "${LEISAAC}/assets/robot/so101_follower.usd"
else
    echo "so101_follower.usd already present, skipping."
fi

# Kitchen scene
if [ ! -d "${LEISAAC}/assets/scenes/kitchen_with_orange" ]; then
    echo "Downloading kitchen_with_orange.zip..."
    TMP=$(mktemp -d)
    curl -L "${RELEASE}/kitchen_with_orange.zip" -o "${TMP}/kitchen_with_orange.zip"
    unzip -q "${TMP}/kitchen_with_orange.zip" -d "${LEISAAC}/assets/scenes/"
    rm -rf "${TMP}"
else
    echo "kitchen_with_orange scene already present, skipping."
fi

echo "Assets ready."
