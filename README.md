# Robot Realtime Control Interfaces

Robots Realtime is a research codebase supporting modular software stacks for realtime control, teleoperation, and policy integration on real-world robot embodiments including bi-manual I2RT YAM arms, Franka Panda, (more to come...).

It provides extensible pythonic infrastructure for low-latency joint command streaming, agent-based policy control, visualization, and integration with inverse kinematics solvers like [pyroki](https://github.com/chungmin99/pyroki) developed by [Chung-Min Kim](https://chungmin99.github.io/)! 

Examples:

<img src="media/yam_realtime.gif" width="500">
<img src="media/franka_realtime2.gif" width="500">
<img src="media/yam_active_leader.gif" width="500">
<!-- ![yam_realtime](media/yam_realtime.gif) -->
<!-- ![franka_realtime](media/franka_realtime.gif) -->
<!-- ![franka_realtime2](media/franka_realtime2.gif) -->

## Installation
Clone the repository and initialize submodules:
```bash
git clone --recurse-submodules https://github.com/uynitsuj/robots_realtime.git
# Or if already cloned without --recurse-submodules, run:
git submodule update --init --recursive
```
Install the main package and I2RT repo for CAN driver interface using uv:
```bash
cd robots_realtime
curl -LsSf https://astral.sh/uv/install.sh | sh
source .venv/bin/activate

uv venv --python 3.11
uv pip install -e .
```
## Configuration

### CAN Interface (YAM arms)
Configure YAM arms CAN chain according to instructions from the [I2RT repo](https://github.com/i2rt-robotics/i2rt).

To avoid needing `sudo` every launch, set up a udev rule that auto-configures CAN interfaces on plug-in:
```bash
echo 'SUBSYSTEM=="net", KERNEL=="can*", ACTION=="add", RUN+="/sbin/ip link set %k up type can bitrate 1000000"' | sudo tee /etc/udev/rules.d/99-can.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
This only needs to be done once. After that, CAN interfaces are brought up automatically at 1000000bps.

### Camera Test
Test connected RealSense cameras with the diagnostic viewer:
```bash
uv run python scripts/test_realsense_cameras.py
```
Interactive picker lets you choose a camera. Additional options:
- `--show-serial` show serial number overlay
- `--depth` show colorized depth stream
- `--flip-ud` / `--flip-lr` flip image
- `r` key to start/stop recording, `s` to toggle serial overlay, `q` to quit

### GELLO Server (Network Teleop)
For network-based GELLO teleop (`yam_gello_network_bimanual.yaml`), the position server must be running on the R1 Lite Teleop onboard computer. A helper script handles kill/copy/start over SSH:
```bash
bash scripts/start_gello_server.sh            # start on default host (10.42.0.1)
bash scripts/start_gello_server.sh 10.42.0.2  # custom host
bash scripts/start_gello_server.sh --kill      # kill remote server
```
The server runs in a detached `screen` session. View logs with `ssh cat@10.42.0.1 'screen -r gello_server'`.

## Launch
Then run the launch entrypoint script with an appropriate robot config file.
For Bimanual YAMS:
```bash
uv run robots_realtime/envs/launch.py --config_path configs/yam/yam_viser_bimanual.yaml
```
For Franka Panda (with default panda gripper):
```bash
uv sync --extra sensors --extra franka_panda
uv run robots_realtime/envs/launch.py --config_path configs/franka/franka_viser_osc.yaml
```
or for robotiq gripper (ensure flange orientation is correct):
```bash
uv run robots_realtime/envs/launch.py --config_path configs/franka/franka_robotiq_client.yaml
```

## VR Teleoperation (Pico)

Control bimanual YAM arms using Pico VR controllers via [XRoboToolkit](https://github.com/XR-Robotics/XRoboToolkit-PC-Service).

### Prerequisites
1. Download and run [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) on your PC.
2. Connect your Pico headset and verify poses are streaming in the PC Service app.

### Install XRoboToolkit SDK
```bash
source .venv/bin/activate
bash scripts/install_xrobotoolkit_sdk.sh
```

### Launch VR Teleop
```bash
uv run robots_realtime/envs/launch.py --config_path configs/yam_vr_bimanual.yaml
```

### Controls
| Button | Function |
|---|---|
| **Grip** (hold) | Activate arm control — arm follows hand movement |
| **Grip** (release) | Freeze arm — re-grip to continue from new position |
| **Trigger** | Gripper control (0 = open, fully pressed = closed) |

The Viser web UI is still active during VR teleop for monitoring the robot state.

## Extending with Custom Agents
To integrate your own controller or policy:

Subclass the base agent interface:
```python
from robots_realtime.agents.agent import Agent

class MyAgent(Agent):
    ...
```
Add your agent to your YAML config so the launcher knows which controller to instantiate.

Examples of agents you might implement:
- Leader arm or VR controller teleoperation
- Learned policy (e.g., Diffusion Policy, ACT, PI0)
- Offline motion-planner + scripted trajectory player

## Linting
If contributing, please use ruff (automatically installed) for linting (https://docs.astral.sh/ruff/tutorial/#getting-started)
```bash
ruff check # lint
ruff check --fix # lint and fix anything fixable
ruff format # code format
```

## Roadmap/Todos

- [ ] Add data logging infrastructure
- [ ] Implement a [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/) agent controller
- [ ] Implement a [Physical Intelligence π0](https://www.physicalintelligence.company/blog/pi0) agent controller
