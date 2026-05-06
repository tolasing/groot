![groot](docs/source/_static/isaaclab.jpg)

---

# Groot

**Groot** is a project to train a Vision-Language-Action (VLA) model for robot manipulation.
The first milestone is bringing three simulation stacks together into a single unified environment:
[NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html),
[Isaac Lab](https://github.com/isaac-sim/IsaacLab), and
[LeIsaac](https://github.com/LightwheelAI/leisaac) — using these as the data generation and training backbone for Groot.

---

## Stack Overview

| Component | Role |
|-----------|------|
| **Isaac Sim 5.1.0** | Physics + sensor simulation engine |
| **Isaac Lab** | RL / IL framework, environments, robot models |
| **LeIsaac** | Lerobot-compatible data collection and policy training inside Isaac Sim |

---

## Prerequisites

- [Docker](https://docs.docker.com/engine/install/) with NVIDIA Container Toolkit
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- NVIDIA GPU with drivers installed

---

## Getting Started

### First time setup

```
Ctrl+Shift+P → Dev Containers: Rebuild and Reopen in Container
```

VS Code will show a picker — choose your profile:

| Option | Includes |
|--------|---------|
| **GROOT — Base** | Isaac Sim + Isaac Lab |
| **GROOT — ROS2** | Isaac Sim + Isaac Lab + ROS2 Humble |
| **GROOT — LeIsaac** | Isaac Sim + Isaac Lab + LeIsaac |

This builds the Docker image and opens the container. Only needed once or after Dockerfile changes.

### Daily use

```
Ctrl+Shift+P → Dev Containers: Reopen in Container
```

Starts the existing container and attaches — no rebuild needed.

---

## Workspace Layout

Inside the container:

```
/groot_ws/              ← VS Code opens here (groot git repo, full git access)
├── src/
│   ├── isaaclab/       ← symlink to /workspace/isaaclab
│   ├── leisaac/        ← symlink to /workspace/leisaac  (LeIsaac only)
│   └── <your code>     ← tracked in this repo, pushable to GitHub
├── .devcontainer/
├── docker/
└── ...
```

> **LeIsaac first launch:** the devcontainer automatically downloads:
> - `so101_follower.usd` → `/workspace/leisaac/assets/robot/`
> - `kitchen_with_orange` scene → `/workspace/leisaac/assets/scenes/`

---

## Branches

| Branch | Isaac Sim | Purpose |
|--------|-----------|---------|
| `main` | 5.1.0 | Stable development |
| `develop` | 6.0 (upstream Isaac Lab) | Testing CloudXR streaming with Meta Quest 3S |

---

## License

The Isaac Lab framework is released under [BSD-3 License](LICENSE).
The `isaaclab_mimic` extension is released under [Apache 2.0](LICENSE-mimic).
Isaac Sim itself is under proprietary licensing — see [Isaac Sim license](docs/licenses/dependencies/isaacsim-license.txt).

---

## Acknowledgement

Isaac Lab originated from the [Orbit](https://isaac-orbit.github.io/) framework.
LeIsaac is developed by [Lightwheel AI](https://github.com/LightwheelAI/leisaac).
