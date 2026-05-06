# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Keyboard teleoperation for Isaac-Forge-NutThread-Direct-v0.

Unlike the manager-based teleop script (teleop_se3_agent.py), this script targets a
DirectRLEnv whose action space is a 7D normalized absolute target, not an SE(3) delta:

  action[0:3] - XYZ position target relative to bolt, normalized [-1, 1]
                (scaled internally by pos_action_bounds = [0.05, 0.05, 0.05] m)
  action[3:5] - Roll / pitch — the env zeros these out, ignored here
  action[5]   - Yaw target normalized [-1, 1]  →  maps to [-180°, +90°]
  action[6]   - Success prediction [-1, 1]  (K key toggles)

The keyboard returns per-step velocity deltas that are accumulated into a running
action_state each simulation step and clamped to [-1, 1].

Key bindings:
  W / S   —  +/- X  (toward / away from bolt)
  A / D   —  +/- Y  (left / right)
  Q / E   —  +/- Z  (up / down)
  C / V   —  +/- Yaw  (thread / un-thread the nut)
  K       —  Toggle success prediction signal
  R       —  Reset environment + action state
  L       —  Re-center action state to zero (keeps env running)

Run:
  python scripts/environments/teleoperation/teleop_forge_nut_thread.py \\
      --task=Isaac-Forge-NutThread-Direct-v0 \\
      --num_envs=1 \\
      --device=cuda \\
      --pos_sensitivity=0.3 \\
      --yaw_sensitivity=0.2
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Keyboard teleoperation for Isaac-Forge-NutThread-Direct-v0.")
parser.add_argument("--task", type=str, default="Isaac-Forge-NutThread-Direct-v0", help="Gym task name.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments.")
parser.add_argument(
    "--pos_sensitivity",
    type=float,
    default=0.3,
    help="Position increment added to the normalized action per sim step while a key is held. "
    "Range [0, 1]. Higher = faster. Default 0.3 is intentionally high for testing.",
)
parser.add_argument(
    "--yaw_sensitivity",
    type=float,
    default=0.2,
    help="Yaw increment added to the normalized action per sim step while C/V is held. "
    "Range [0, 1]. Default 0.2 sweeps the full 270° range in ~10 steps.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import logging

import gymnasium as gym
import torch

from isaaclab.devices import Se3Keyboard, Se3KeyboardCfg

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

logger = logging.getLogger(__name__)


def main() -> None:
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)

    # Disable episode timeout so the environment never auto-resets during teleop.
    env_cfg.episode_length_s = 1e9

    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # Se3Keyboard is used purely for key-state detection.
    # pos_sensitivity and rot_sensitivity become the per-step delta added to action_state.
    keyboard = Se3Keyboard(
        Se3KeyboardCfg(
            pos_sensitivity=args_cli.pos_sensitivity,
            rot_sensitivity=args_cli.yaw_sensitivity,
            gripper_term=True,
            sim_device=args_cli.device,
        )
    )

    # Mutable flags shared with key callbacks.
    flags = {"reset": False, "recenter": False}

    def on_reset():
        flags["reset"] = True

    def on_recenter():
        flags["recenter"] = True

    # R → full env reset.  L → re-center action state only (L also resets keyboard deltas internally).
    keyboard.add_callback("R", on_reset)
    keyboard.add_callback("L", on_recenter)

    # action_state is the running 7D command accumulated from keyboard deltas.
    # Initialized to zero; action[6] = -1 means "not predicting success yet".
    action_state = torch.zeros(7, dtype=torch.float32, device=args_cli.device)
    action_state[6] = -1.0

    env.reset()
    keyboard.reset()

    print("\n=== Forge NutThread Keyboard Teleop ===")
    print(f"  pos_sensitivity : {args_cli.pos_sensitivity}  (action units / step while key held)")
    print(f"  yaw_sensitivity : {args_cli.yaw_sensitivity}  (action units / step while C/V held)")
    print()
    print("  W / S  →  +/- X  (toward / away from bolt)")
    print("  A / D  →  +/- Y  (left / right)")
    print("  Q / E  →  +/- Z  (up / down)")
    print("  C / V  →  +/- Yaw  (thread / un-thread)")
    print("  K      →  Toggle success prediction")
    print("  R      →  Reset environment")
    print("  L      →  Re-center action (no env reset)")
    print("=======================================\n")

    while simulation_app.is_running():
        with torch.inference_mode():
            # advance() returns [dx, dy, dz, rot_vec_x, rot_vec_y, rot_vec_z, gripper]
            # While a key is held the corresponding component equals ±sensitivity each call.
            kb = keyboard.advance()

            # Accumulate XYZ into action_state[0:3].
            action_state[0:3] += kb[0:3]

            # Accumulate yaw-only into action_state[5].
            # C/V produce rot_vec[2] ≈ ±yaw_sensitivity; roll/pitch are zeroed by the env anyway.
            action_state[5] += kb[5]

            # K toggles gripper term → use as success-prediction signal.
            action_state[6] = kb[6]

            # Keep action_state inside the valid normalized range.
            action_state.clamp_(-1.0, 1.0)

            actions = action_state.unsqueeze(0).expand(env.num_envs, -1)
            env.step(actions)

            if flags["reset"]:
                env.reset()
                keyboard.reset()
                action_state.zero_()
                action_state[6] = -1.0
                flags["reset"] = False
                print("Environment reset.")

            if flags["recenter"]:
                action_state.zero_()
                action_state[6] = -1.0
                flags["recenter"] = False
                print("Action state re-centered.")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
