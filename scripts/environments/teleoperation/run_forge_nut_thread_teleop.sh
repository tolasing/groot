#!/usr/bin/env bash
python scripts/environments/teleoperation/teleop_forge_nut_thread.py \
    --task=Isaac-Forge-NutThread-Direct-v0 \
    --num_envs=1 \
    --device=cuda \
    --enable_cameras \
    --livestream 2
