# Passive Fault-Tolerant Control of Unmanned Underwater Vehicles (UUV)

> A ROS2-based nonlinear control framework for autonomous underwater vehicles featuring passive fault tolerance through thruster redundancy, Lyapunov-stable backstepping, and Finite-Time Sliding Mode Control — validated in high-fidelity Stonefish simulation.

---

## Overview

Underwater vehicles operating in unstructured ocean environments face a fundamental reliability challenge: **thruster failures**. This project implements a **passive fault-tolerant control (FTC)** system that maintains trajectory tracking performance in the presence of thruster faults — without requiring explicit fault detection or controller reconfiguration.

The framework is built around two nonlinear control strategies:

- **Backstepping Control** — Lyapunov-based recursive design guaranteeing asymptotic stability
- **Finite-Time Sliding Mode Control (FTSMC)** — Robust sliding mode controller achieving finite-time convergence with chattering suppression via boundary layer saturation

Both controllers operate on a full **6-DOF hydrodynamic model** formulated using Fossen's equations of motion, with passive fault accommodation handled at the thruster allocation layer via pseudo-inverse redistribution.

Tested on the **BlueROV2 Heavy** (8-thruster vectored configuration) inside the **Stonefish** underwater physics simulator.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Stonefish Simulator                          │
│         Hydrodynamics | Thruster Dynamics | Sensor Simulation        │
│               (IMU | DVL | Pressure | Stereo Camera)                 │
└────────────────────────┬─────────────────────────────────────────────┘
           Sensor Data   │                        ▲  PWM Commands
                         ▼                        │
┌──────────────────────────────────────────────────────────────────────┐
│                        FTC Control Stack                              │
│                                                                      │
│   ┌──────────────────┐     ┌──────────────────────────────────────┐  │
│   │  trajectory_node │────▶│           controller_node            │  │
│   │  Desired State   │     │  Backstepping  │  FTSMC  │  I-FTSMC  │  │
│   └──────────────────┘     └──────────────────┬───────────────────┘  │
│                                               │  τ (6-DOF forces)    │
│   ┌──────────────────┐                        ▼                      │
│   │state_estimator   │────▶  ┌────────────────────────────────────┐  │
│   │ Ground Truth /   │       │     thruster_allocator_node        │  │
│   │ Sensor Fusion    │       │  B† pseudo-inverse | Fault Handling │  │
│   └──────────────────┘       └──────────────────┬─────────────────┘  │
│                                                 │                    │
│                              ┌──────────────────▼─────────────────┐  │
│                              │         data_logger_node           │  │
│                              │      CSV Logging | 10 Hz           │  │
│                              └────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
Passive-Fault-Tolerant-Control-UUV/
└── src/
    ├── ftc_control/               # Core control system (C++17)
    │   ├── include/ftc_control/
    │   │   ├── controllers/       # backstepping, ftsmc, i_ftsmc
    │   │   ├── models/            # 6-DOF AUV dynamics (Fossen)
    │   │   ├── allocation/        # Pseudo-inverse thruster allocation
    │   │   └── utils/             # Math utilities, frame transforms
    │   ├── src/
    │   │   ├── controllers/
    │   │   ├── models/
    │   │   ├── allocation/
    │   │   └── nodes/             # ROS2 executable nodes
    │   ├── config/                # Controller gain YAML files
    │   └── launch/                # ROS2 launch files
    └── ftc_simulation/            # Stonefish simulation environment
        ├── scenarios/             # World + BlueROV2 robot definition (.scn)
        ├── data/bluerov2/         # 3D mesh and texture assets
        ├── config/                # Simulation parameters
        └── launch/                # Full simulation launch files
```

---

## Workspace Setup

This repo contains only the ROS2 source packages. The full working environment requires the **Stonefish C++ library** and the **stonefish_ros2** ROS2 interface to be set up alongside it. The recommended workspace layout mirrors the project's development structure:

```
project_root/                          # Your working directory
│
├── stonefish/                         # Stonefish C++ physics library
│   └── (cloned from patrykcieslak/stonefish — see Prerequisites)
│
└── Passive-Fault-Tolerant-Control-UUV/   # ← this repo (also the colcon workspace)
    └── src/
        ├── ftc_control/                 # Control algorithms and ROS2 nodes
        ├── ftc_simulation/              # Stonefish simulation setup
        └── stonefish_ros2/              # (cloned from patrykcieslak/stonefish_ros2)
```

---

## Prerequisites

### 1. ROS2 Humble
```bash
# Ubuntu 22.04 — follow the official installation guide:
# https://docs.ros.org/en/humble/Installation.html
source /opt/ros/humble/setup.bash
```

---

### 2. Stonefish — Underwater Physics Simulation Library

**What it is:** Stonefish is a C++ physics simulation library specifically designed for marine robotics. It provides real-time hydrodynamics (added mass, drag, buoyancy), accurate thruster dynamics, sensor simulation (IMU, DVL, pressure, cameras, sonar), and underwater lighting effects.

**Repository:** https://github.com/patrykcieslak/stonefish

This is the **core simulation engine** and must be compiled and installed system-wide before building the ROS2 interface.

```bash
# Install system dependencies
sudo apt install libglm-dev libsdl2-dev

# Clone and build
mkdir project_root && cd project_root
git clone https://github.com/patrykcieslak/stonefish.git
cd stonefish
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

### 3. stonefish_ros2 — ROS2 Interface for Stonefish

**What it is:** stonefish_ros2 is the ROS2 wrapper around the Stonefish library. It provides the ROS2 node that runs the simulator, parses `.scn` scenario files, publishes sensor topics, and subscribes to actuator commands. This project's `ftc_simulation` package depends on it at runtime.

**Repository:** https://github.com/patrykcieslak/stonefish_ros2

This is a **separate ROS2 package** that must be cloned into your workspace alongside this repo — it is not bundled here.

```bash
# Clone into your workspace src directory
cd project_root/Passive-Fault-Tolerant-Control-UUV/src
git clone https://github.com/patrykcieslak/stonefish_ros2.git
```

---

### 4. System Dependencies
```bash
sudo apt install \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  libeigen3-dev
```

---

## Installation

```bash
# Clone this repo (it is the colcon workspace)
cd project_root
git clone https://github.com/YOUR_USERNAME/Passive-Fault-Tolerant-Control-UUV.git

# Clone stonefish_ros2 into the workspace
cd Passive-Fault-Tolerant-Control-UUV/src
git clone https://github.com/patrykcieslak/stonefish_ros2.git

# Build the workspace
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Usage

### Launch Full Simulation
```bash
# Default: BlueROV2 | Backstepping controller | Hover trajectory
ros2 launch ftc_simulation full_simulation.launch.py
```

### Select Controller and Trajectory
```bash
# FTSMC controller with 3D figure-eight trajectory
ros2 launch ftc_simulation full_simulation.launch.py \
  controller_type:=ftsmc trajectory_type:=eight

# Backstepping with lawnmower survey pattern
ros2 launch ftc_simulation full_simulation.launch.py \
  controller_type:=backstepping trajectory_type:=lawnmower

# Headless mode (no GUI window)
ros2 launch ftc_simulation full_simulation.launch.py \
  window_width:=0 window_height:=0
```

### Launch Arguments

| Argument | Options | Default |
|----------|---------|---------|
| `controller_type` | `backstepping`, `ftsmc`, `i_ftsmc` | `backstepping` |
| `trajectory_type` | `hover`, `eight`, `lawnmower`, `waypoint` | `hover` |
| `robot_name` | any string | `BLUEROV2` |
| `robot_type` | `bluerov2`, `girona500` | `bluerov2` |
| `window_width` | integer (0 = headless) | `1920` |
| `window_height` | integer (0 = headless) | `1080` |

### Simulating Thruster Faults
```bash
# Thruster 1 at 50% efficiency, Thruster 3 fully failed
ros2 topic pub /FTC/thruster_faults std_msgs/msg/Float64MultiArray \
  "data: [0.5, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
```
Values: `1.0` = healthy, `0.0` = complete failure, `(0, 1)` = partial degradation.

---

## ROS2 Interface

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/FTC/desired_state` | `nav_msgs/Odometry` | Published | Reference trajectory |
| `/FTC/estimated_state` | `nav_msgs/Odometry` | Published | Current vehicle state |
| `/FTC/control_forces` | `std_msgs/Float64MultiArray` | Published | 6-DOF wrench [X,Y,Z,K,M,N] |
| `/FTC/thruster_faults` | `std_msgs/Float64MultiArray` | Subscribed | Per-thruster health vector |
| `/<robot_name>/setpoint/pwm` | `std_msgs/Float64MultiArray` | Published | Thruster PWM to simulator |

---

## Controllers

### Backstepping
A Lyapunov-based recursive nonlinear controller. Virtual control laws are designed at each step to ensure stability of the cascaded system. Guarantees asymptotic tracking under smooth trajectories.

- Gain matrices `K1`, `K2` tunable via `config/backstepping.yaml`
- Suitable for nominal operating conditions

### Finite-Time Sliding Mode Control (FTSMC)
Sliding mode controller with a PD-structured sliding surface `s = ė_ν + k₁·sig(e_ν) + k₂·e_ν` and finite-time reaching law using a `sig(x, β) = |x|^β · sgn(x)` nonlinearity for finite-time convergence.

- PD sliding surface for improved transient response and reduced overshoot
- Parameters tunable per DOF for surge/sway/heave vs roll/pitch/yaw
- Full 6-DOF Fossen hydrodynamic model in the control law (M, C, D, g)
- Robust against parametric uncertainty and external disturbances

### Integral Finite-Time Sliding Mode Control (I-FTSMC)
FTSMC variant with a PI-Sig sliding surface that adds an integral term with anti-windup clamping for improved steady-state accuracy.

- PI-Sig sliding surface: `s = e_ν + K_i ∫(e_ν + Γ·sig(e_ν))dt`
- Kinematic-dynamic cascade: position error drives a virtual velocity command, velocity error feeds the sliding surface
- Better steady-state performance than standard FTSMC at the cost of added integral state
- Robust against parametric uncertainty and external disturbances

---

## Vehicle: BlueROV2 Heavy

| Parameter | Value |
|-----------|-------|
| Mass (in air) | 11.5 kg |
| Thrusters | 8x Blue Robotics T200 |
| Configuration | 4 horizontal (vectored ±45°) + 4 vertical |
| Max thrust per thruster | ~34 N at 16V |
| Added mass | [5.5, 12.7, 14.6, 0.12, 0.12, 0.12] kg / kg·m² |
| Linear damping | [4.0, 6.2, 5.2, 0.07, 0.07, 0.07] |
| Quadratic damping | [18.2, 21.7, 36.8, 1.5, 1.5, 1.5] |
| Controllable DOF | 6 (surge, sway, heave, roll, pitch, yaw) |

---

## Third-Party Credits

- **BlueROV2 mesh assets** — sourced from [stonefish_bluerov2](https://github.com/bvibhav/stonefish_bluerov2) by bvibhav, licensed under Apache-2.0
- **Stonefish simulator** — developed by Patryk Cieślak ([stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2)), licensed under GPL-3.0

See [NOTICE](./NOTICE) for full attribution details.

If you use Stonefish in your research, please cite:
```bibtex
@inproceedings{stonefish,
  author    = {Cieślak, Patryk},
  booktitle = {OCEANS 2019 - Marseille},
  title     = {Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface},
  year      = {2019},
  doi       = {10.1109/OCEANSE.2019.8867434}
}
```

---

## License

This project is licensed under the **MIT License**.
