![groot](docs/source/_static/isaaclab.jpg)

---

# Groot

**Groot** is a project to train a Vision-Language-Action (VLA) model for robot manipulation.
The first milestone is bringing three simulation stacks together into a single unified environment:

1.[NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)

2.[Isaac Lab](https://github.com/isaac-sim/IsaacLab)

3.[LeIsaac](https://github.com/LightwheelAI/leisaac) — using these as the data generation and training backbone for Groot.

---

## Table of Contents

- [Stack Overview](#stack-overview)
- [Prerequisites](#prerequisites)
  - [Install Docker](#install-docker)
  - [Install NVIDIA Container Toolkit](#install-nvidia-container-toolkit)
- [Getting Started](#getting-started)
- [Workspace Layout](#workspace-layout)
- [Branches](#branches)
- [Gamepad Teleoperation](#gamepad-teleoperation)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Acknowledgement](#acknowledgement)

---

## Stack Overview

| Component | Role |
|-----------|------|
| **Isaac Sim 5.1.0** | Physics + sensor simulation engine |
| **Isaac Lab** | RL / IL framework, environments, robot models |
| **LeIsaac** | Lerobot-compatible data collection and policy training inside Isaac Sim |

---

## Prerequisites

- Docker (see install steps below) with NVIDIA Container Toolkit
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- NVIDIA GPU with drivers installed

### Install Docker

**Using the convenience script:**

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

**Post-install steps:**

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

**Verify:**

```bash
docker run hello-world
```

### Install NVIDIA Container Toolkit

**Configure the repository:**

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update
```

**Install the packages:**

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

**Configure the container runtime:**

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

**Verify:**

```bash
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

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

## Gamepad Teleoperation

Gamepad input is forwarded from your local machine to the cloud instance over SSH using USB/IP.

### 1. On your laptop — share the gamepad

Find your gamepad's bus ID:

```bash
usbip list -l
```

Look for your controller in the output (e.g. `Xbox 360 Controller`). Note the bus ID (e.g. `5-1`).

```bash
sudo modprobe usbip-core
sudo modprobe usbip-host
sudo usbip bind --busid 5-1          # replace 5-1 with your actual bus ID
sudo nohup usbipd > /tmp/usbipd.log 2>&1 &
```

Then open the SSH tunnel (keep this terminal open):

```bash
ssh -R 3240:localhost:3240 root@<cloud-ip>
```

### 2. On the cloud machine — attach the device

```bash
sudo usbip attach -r 127.0.0.1 -b 5-1    # replace 5-1 with the same bus ID
```

Verify the gamepad is visible:

```bash
ls /dev/input/    # should show js0
```

> **Note:** The bus ID (`5-1`) may differ on your machine. Always check with `usbip list -l` first.  
> If the gamepad was attached *after* the container was started, recreate the container so the device is whitelisted in the kernel cgroup:
> ```bash
> docker rm -f isaac-lab-leisaac
> # then re-run the docker run command with --device /dev/input
> ```

### 3. Run teleoperation

```bash
docker exec -it isaac-lab-leisaac bash -c "cd /workspace/isaaclab && \
  ./isaaclab.sh -p /groot_ws/scripts/environments/teleoperation/teleop_se3_agent.py \
  --task=Isaac-Deploy-GearAssembly-UR10e-2F140-v0 \
  --teleop_device=gamepad \
  --num_envs=1 --device=cuda --livestream 2"
```

For LeIsaac tasks (SO-101 robot):

```bash
docker exec -it isaac-lab-leisaac bash -c "cd /workspace/leisaac && \
  /workspace/isaaclab/isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task=LeIsaac-SO101-PickOrange-v0 \
  --teleop_device=gamepad \
  --num_envs=1 --device=cuda --livestream 2"
```

---

## Troubleshooting

### `dpkg` lock held by `unattended-upgr`

**Error:**

```
E: Could not get lock /var/lib/dpkg/lock-frontend. It is held by process <PID> (unattended-upgr)
E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), is another process using it?
```

**Cause:** Ubuntu's `unattended-upgrades` service is running in the background and holds the package manager lock.

**Fix:** Wait for it to finish, then retry:

```bash
sudo wait-for-it /var/lib/dpkg/lock-frontend; sudo apt-get install -y <package>
```

Or wait manually:

```bash
while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do sleep 3; done
```

---

## License

The Isaac Lab framework is released under [BSD-3 License](LICENSE).
The `isaaclab_mimic` extension is released under [Apache 2.0](LICENSE-mimic).
Isaac Sim itself is under proprietary licensing — see [Isaac Sim license](docs/licenses/dependencies/isaacsim-license.txt).

---

## Acknowledgement

Isaac Lab originated from the [Orbit](https://isaac-orbit.github.io/) framework.
LeIsaac is developed by [Lightwheel AI](https://github.com/LightwheelAI/leisaac).
