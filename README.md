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

## Getting Started with Docker

The repo ships three Docker profiles — `base`, `ros2`, and `leisaac`. Use `container.py` to build and start whichever you need.

### 1. Build an image

```bash
python docker/container.py start base      # Isaac Sim + Isaac Lab only
python docker/container.py start ros2      # adds ROS2 Humble
python docker/container.py start leisaac   # adds LeIsaac on top of base
```

### 2. Enter the container

```bash
python docker/container.py enter base      # or ros2 / leisaac
```

### 3. Stop the container

```bash
python docker/container.py stop base       # or ros2 / leisaac
```

---

## Dev Container (VS Code)

A devcontainer is configured for each profile under `.devcontainer/`. Open the repo in VS Code and run:

**`Ctrl+Shift+P` → Dev Containers: Reopen in Container**

VS Code will show a picker — choose **Base**, **ROS2**, or **LeIsaac**. The container opens directly into `/workspace`.

> On first launch of the **LeIsaac** devcontainer, a `postCreateCommand` automatically downloads and places the LeIsaac assets:
> - `so101_follower.usd` → `/workspace/leisaac/assets/robot/`
> - `kitchen_with_orange` scene → `/workspace/leisaac/assets/scenes/`

---

## Isaac Sim Version

| Branch / Version | Isaac Sim |
|------------------|-----------|
| `main`           | 5.1.0     |

---

## License

The Isaac Lab framework is released under [BSD-3 License](LICENSE).
The `isaaclab_mimic` extension is released under [Apache 2.0](LICENSE-mimic).
Isaac Sim itself is under proprietary licensing — see [Isaac Sim license](docs/licenses/dependencies/isaacsim-license.txt).

---

## Acknowledgement

Isaac Lab originated from the [Orbit](https://isaac-orbit.github.io/) framework.
LeIsaac is developed by [Lightwheel AI](https://github.com/LightwheelAI/leisaac).
