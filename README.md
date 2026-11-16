# EU4M TurtleBot3 – Autonomous Lunar Rover

## Project Overview

This repository contains the ROS 2 software stack, configuration files, simulation environment, and development tools for an autonomous rover based on the **TurtleBot3 Burger** platform.

The project is developed for the **VP774A Design and Development Project II** course at **Högskolan i Skövde (University of Skövde), Sweden**, during WiSe 2026.

The current system supports two development modes using the same ROS 2 workspace and Docker image:

- **Simulation mode** using Gazebo and ROS 2 locally.
- **Real robot mode** using a physical TurtleBot3 connected through Zenoh.

The objective is to develop and validate the autonomy software in simulation first and later deploy the same ROS interfaces and algorithms on the physical robot.

---

# Project Scenario

The rover will operate in a maze containing dynamic gates.

The project is divided into two main phases.

### Phase 1 – Mapping

The TurtleBot3 explores the maze while all gates are open.

The robot must generate a map of the environment using its LiDAR and odometry.

Planned components include:

- Gazebo simulation
- TurtleBot3 Burger
- LiDAR
- SLAM Toolbox
- ROS 2 TF
- Odometry
- Map generation and storage

### Phase 2 – Maze Navigation

The robot must use the previously generated map to reach the goal as quickly as possible.

During this phase, gates may close and block previously available corridors.

The navigation system will therefore need to:

- Localize the robot in the existing map.
- Plan a route toward the goal.
- Detect newly blocked corridors.
- Replan when gates close.
- Avoid static and dynamic obstacles.
- Prevent the robot from entering or remaining underneath a closing gate.
- Minimize the total time required to reach the goal.

Planned components include:

- Nav2
- Dynamic costmaps
- Obstacle detection
- Gate state management
- Route replanning
- Collision monitoring
- Mission management

---

# System Architecture

The physical TurtleBot3 runs ROS 2 Jazzy directly on its Raspberry Pi.

The Raspberry Pi also hosts the Zenoh router used by remote development computers.

```text
                    Local Wi-Fi Network
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼                           ▼
      TurtleBot3 / Pi             Development PC
       ROS 2 Jazzy                 ROS 2 Jazzy
             │                           │
      rmw_zenoh_cpp                rmw_zenoh_cpp
             │                           │
      Zenoh Router :7447  ◄──────────────┘
             │
      OpenCR + LiDAR
```

The TurtleBot is advertised through mDNS as:

```text
turtlebot3.local
```

Therefore, the robot does **not** require a fixed IP address.

The Raspberry Pi may receive a different address after reboot and development computers can still discover it automatically.

---

# Hardware

Current hardware includes:

- **Base Platform:** TurtleBot3 Burger
- **Actuators:** 2 × DYNAMIXEL XL430 motors
- **Embedded Controller:** OpenCR 1.0
- **Compute Node:** Raspberry Pi
- **Perception:** 2D 360° LDS LiDAR
- **Camera:** Raspberry Pi Camera Module v2 with Sony IMX219 8 MP sensor

Camera integration is separate from the current LiDAR/navigation stack and may later use packages such as `camera_ros` or `v4l2_camera`.

---

# ROS 2 Topics

The physical TurtleBot currently provides the following main topics:

```text
/battery_state
/cmd_vel
/imu
/joint_states
/magnetic_field
/odom
/parameter_events
/robot_description
/rosout
/scan
/sensor_state
/tf
/tf_static
```

The same main interfaces will also be used during simulation whenever possible.

This allows higher-level software to remain independent of whether the data originates from Gazebo or the real robot.

```text
Simulation                    Real Robot
----------                    ----------
/scan          <----------->  /scan
/odom          <----------->  /odom
/tf            <----------->  /tf
/cmd_vel       <----------->  /cmd_vel
```

---

# Docker Environment

The Docker image provides the ROS 2 Jazzy development environment, including tools for:

- RViz2
- Gazebo
- TurtleBot3
- SLAM Toolbox
- Nav2
- Behavior Trees
- MoveIt 2
- ROS-Gazebo integration

The same Docker image is used for both simulation and real-robot development.

The runtime mode is selected through environment variables defined by `compose.yaml`.

---

# Repository Structure

The main project structure is:

```text
TurtleBot_Rover/
├── compose.yaml
├── docker/
│   └── Dockerfile
├── scripts/
│   └── start.sh
├── ros2_ws/
│   ├── config/
│   │   ├── rviz/
│   │   └── zenoh/
│   └── src/
└── README.md
```

The ROS 2 workspace is mounted inside Docker as:

```text
./ros2_ws -> /ros2_ws
```

Therefore, files inside `ros2_ws` remain stored in the repository even if the container is deleted or recreated.

---

# Windows + WSL2 + Docker

## Requirements

Install:

- Windows 11
- WSL2
- Ubuntu under WSL
- Docker Desktop
- Docker Desktop WSL integration enabled

Verify from WSL:

```bash
docker --version
docker compose version
```

---

# Clone the Repository

```bash
git clone https://github.com/LuisAntonio1929/TurtleBot_Rover.git
cd TurtleBot_Rover
```

If already cloned:

```bash
git pull
```

---

# Build the Docker Image

A build is required the first time:

```bash
docker compose build
```

A rebuild is normally required only when:

- `docker/Dockerfile` changes.
- New Ubuntu packages are added.
- New ROS 2 packages are added through APT.
- System-level dependencies change.

A clean rebuild can be performed with:

```bash
docker compose build --no-cache
```

Changes to source files, scripts, RViz configurations, or files mounted through `ros2_ws` normally do **not** require rebuilding the image.

---

# Simulation Mode

Simulation is the default Docker mode.

It uses:

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
TURTLEBOT3_MODEL=burger
ZENOH_SESSION_CONFIG_URI=
```

The TurtleBot3 model is configured automatically through `compose.yaml`.

There is no need to manually run:

```bash
export TURTLEBOT3_MODEL=burger
```

## Start Simulation Mode

From WSL, in the repository root:

```bash
docker compose up -d --force-recreate ros2
```

Enter the container:

```bash
docker compose exec ros2 bash
```

Verify the runtime mode:

```bash
echo $RMW_IMPLEMENTATION
echo $TURTLEBOT3_MODEL
echo $ZENOH_SESSION_CONFIG_URI
```

Expected result:

```text
rmw_fastrtps_cpp
burger

```

The Zenoh configuration should be empty in simulation mode.

---

# Start the Standard TurtleBot3 Simulation

Inside the container:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In another container terminal, teleoperation can be started with:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Verify ROS 2 topics:

```bash
ros2 topic list
```

Typical simulated interfaces should include:

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
/joint_states
```

---

# Real Robot Mode

The real TurtleBot uses **Zenoh** instead of relying on ROS 2 DDS multicast across WSL, Docker, and the local network.

The Zenoh router runs directly on the Raspberry Pi.

Real robot mode uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_SESSION_CONFIG_URI=/config/zenoh/client.json5
TURTLEBOT3_MODEL=burger
```

Do not start real-robot mode using a plain `docker compose up`.

Use:

```bash
./scripts/start.sh
```

The script automatically:

1. Searches for `turtlebot3.local`.
2. Determines the current TurtleBot IP address.
3. Verifies the Zenoh router on TCP port `7447`.
4. Updates the local `.env` file.
5. Configures `turtlebot3.local` inside Docker.
6. Starts the container using `rmw_zenoh_cpp`.
7. Verifies Docker-to-TurtleBot connectivity.

A successful startup should look similar to:

```text
========================================
 TurtleBot Rover - Real Robot Mode
========================================

Searching for turtlebot3.local...
TurtleBot3 found: 192.168.x.x
Zenoh router reachable on port 7447.

Starting Docker in REAL ROBOT mode...

Docker -> TurtleBot3 Zenoh connection OK.

========================================
 TurtleBot Rover ready
========================================
```

Enter the container:

```bash
docker compose exec ros2 bash
```

Verify:

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_zenoh_cpp
```

Then:

```bash
ros2 topic list --no-daemon
```

The physical TurtleBot topics should appear.

---

# Switching Between Simulation and Real Robot

No Docker rebuild is required when changing modes.

## Real Robot → Simulation

Run:

```bash
docker compose up -d --force-recreate ros2
```

The container is recreated using:

```text
rmw_fastrtps_cpp
```

## Simulation → Real Robot

Run:

```bash
./scripts/start.sh
```

The same container image is recreated using:

```text
rmw_zenoh_cpp
```

The architecture is therefore:

```text
                     SAME DOCKER IMAGE
                            │
                     SAME compose.yaml
                            │
                       SAME ros2_ws
                            │
              ┌─────────────┴─────────────┐
              │                           │
         SIMULATION                  REAL ROBOT
              │                           │
 docker compose up             ./scripts/start.sh
              │                           │
 rmw_fastrtps_cpp                 rmw_zenoh_cpp
              │                           │
           Gazebo                  turtlebot3.local
              │                           │
      simulated topics              real topics
              └─────────────┬─────────────┘
                            │
                    Project Software
```

---

# RViz2

A shared RViz configuration is stored at:

```text
ros2_ws/config/rviz/turtlebot3.rviz
```

## Docker

Inside the container:

```bash
rviz2 -d /ros2_ws/config/rviz/turtlebot3.rviz
```

The configuration includes visualization of:

- TurtleBot3 URDF
- `/robot_description`
- TF2
- LiDAR `/scan`
- Odometry-related frames

Because `ros2_ws` is bind-mounted into Docker, changes to the `.rviz` file do **not** require rebuilding the Docker image.

---

# Native Linux / Dual Boot

Users running Ubuntu natively do not need Docker to communicate with the real TurtleBot.

Recommended environment:

- Ubuntu 24.04
- ROS 2 Jazzy

Install:

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-rmw-zenoh-cpp \
  avahi-utils \
  libnss-mdns \
  netcat-openbsd
```

---

## Discover the TurtleBot

The computer and TurtleBot must be connected to the same local network.

Run:

```bash
ping -c 1 turtlebot3.local
```

Verify Zenoh:

```bash
nc -vz turtlebot3.local 7447
```

Expected:

```text
Connection to turtlebot3.local (...) 7447 port [tcp/*] succeeded!
```

---

## Configure Zenoh

Create:

```bash
mkdir -p ~/.config/zenoh
```

Then:

```bash
cat > ~/.config/zenoh/client.json5 <<'EOF'
{
  mode: "client",
  connect: {
    endpoints: [
      "tcp/turtlebot3.local:7447"
    ]
  }
}
EOF
```

Configure the ROS 2 terminal:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$HOME/.config/zenoh/client.json5"
```

Verify:

```bash
ros2 topic list --no-daemon
```

---

# RViz2 on Native Linux

Clone or update the repository:

```bash
git clone https://github.com/LuisAntonio1929/TurtleBot_Rover.git
cd TurtleBot_Rover
```

or:

```bash
git pull
```

From the repository root:

```bash
rviz2 -d "$PWD/ros2_ws/config/rviz/turtlebot3.rviz"
```

The local system should also contain the TurtleBot3 description package:

```bash
ros2 pkg prefix turtlebot3_description
```

If missing:

```bash
sudo apt install ros-jazzy-turtlebot3-description
```

---

# Multiple Computers

Multiple development computers can connect to the same physical TurtleBot simultaneously.

```text
                       TurtleBot3
                    Raspberry Pi
                 Zenoh Router :7447
                    /      |      \
                   /       |       \
                  ▼        ▼        ▼
            Docker PC   Linux PC   Linux PC
```

Each computer communicates directly with the Zenoh router running on the Raspberry Pi.

This configuration has been successfully tested with multiple computers simultaneously accessing the TurtleBot ROS 2 topics.

---

# Network Requirements

The development computer and TurtleBot must be connected to a network that permits direct communication between clients.

Private Wi-Fi networks and personal hotspots normally work correctly.

Some university, guest, or public Wi-Fi networks may use **client isolation**, which can prevent peer-to-peer communication even if both devices have Internet access.

The robot is discovered using:

```text
turtlebot3.local
```

and Zenoh listens on:

```text
TCP 7447
```

---

# Raspberry Pi Startup

The Raspberry Pi automatically starts the required real-robot services.

The main systemd services are:

```text
zenoh-router.service
turtlebot3-ros2.service
```

The startup process is:

```text
Raspberry Pi boot
       ↓
Network available
       ↓
Zenoh Router starts
       ↓
TurtleBot3 ROS 2 bringup starts
       ↓
OpenCR + LiDAR + ROS 2 topics available
```

Under normal operation, users do not need to SSH into the Raspberry Pi to start ROS manually.

---

# SSH Access to the TurtleBot

When needed:

```bash
ssh ubuntu@turtlebot3.local
```

The robot IP does not need to be known manually.

---

# Updating the Repository

Always update the repository before starting development:

```bash
git pull
```

## Changes that normally do NOT require a Docker rebuild

Examples:

```text
README.md
scripts/*
ros2_ws/config/*
ros2_ws/src/*
```

Configuration changes and source files mounted through `/ros2_ws` are immediately available to the container.

ROS packages inside `ros2_ws/src` may still require:

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

but this is different from rebuilding the Docker image.

## Changes that DO require a Docker rebuild

Examples:

- `docker/Dockerfile`
- New system packages
- New ROS packages installed through APT
- New system-level dependencies

Run:

```bash
docker compose build
```

If a local Docker image behaves inconsistently after an update:

```bash
docker compose down
docker compose build --no-cache
```

Then start the desired runtime mode again.

---

# Common Troubleshooting

## `turtlebot3.local` cannot be resolved

Check:

```bash
ping turtlebot3.local
```

Verify:

- TurtleBot3 is powered on.
- Both devices are on the same network.
- The network allows peer-to-peer communication.
- mDNS is available.

---

## Zenoh cannot be reached

Check:

```bash
nc -vz turtlebot3.local 7447
```

If this fails, troubleshoot the network before ROS 2.

---

## Windows can reach the robot but Docker cannot

Run:

```bash
docker compose exec -T ros2 getent hosts turtlebot3.local
```

and:

```bash
docker compose exec -T ros2 nc -vz turtlebot3.local 7447
```

The hostname should resolve to the same IP discovered by `start.sh`.

---

## `nc` is missing inside Docker

Check:

```bash
docker compose exec -T ros2 which nc
```

If it is missing, rebuild the image:

```bash
docker compose down
docker compose build --no-cache
```

---

## ROS 2 topics do not appear in real robot mode

Check:

```bash
echo $RMW_IMPLEMENTATION
echo $ZENOH_SESSION_CONFIG_URI
echo $ROS_DOMAIN_ID
```

Expected:

```text
rmw_zenoh_cpp
/config/zenoh/client.json5
30
```

Then:

```bash
ros2 topic list --no-daemon
```

---

# Development Workflow Summary

## Simulation

```bash
git pull

docker compose up -d --force-recreate ros2
docker compose exec ros2 bash

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Real Robot with Docker

```bash
git pull

./scripts/start.sh
docker compose exec ros2 bash

ros2 topic list --no-daemon
rviz2 -d /ros2_ws/config/rviz/turtlebot3.rviz
```

## Real Robot with Native Linux

```bash
git pull

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$HOME/.config/zenoh/client.json5"

ros2 topic list --no-daemon

rviz2 -d "$PWD/ros2_ws/config/rviz/turtlebot3.rviz"
```

---

# Custom TurtleBot3 Maze Simulation and Autonomy

This section documents the custom ROS 2 package developed for the autonomous maze navigation project. It complements the existing Docker, Zenoh, hardware, and networking documentation.

The simulation and autonomy stack are implemented in:

`/ros2_ws/src/turtlebot_maze_sim/`

The project is divided into two operational stages:

- **Stage 1 — Autonomous Mapping:** The TurtleBot3 explores an initially unknown, static maze and constructs an occupancy grid using SLAM Toolbox.
- **Stage 2 — Goal-Directed Navigation:** The robot localizes itself in a known map and navigates toward a user-defined goal. The environment can include moving gates and additional obstacles that are absent from the static map.

Both stages share the simulated TurtleBot3 platform, LiDAR, wheel odometry, IMU, and Extended Kalman Filter (EKF).

The main distinction is the source of global localization: SLAM Toolbox during mapping and Adaptive Monte Carlo Localization (AMCL) during navigation.

## Package Organization

The `turtlebot_maze_sim` package has the following structure:

```text
turtlebot_maze_sim/
├── CMakeLists.txt
├── LICENSE
├── config/
│   ├── bridges/
│   │   ├── gates_bridge.yaml
│   │   ├── gazebo_bridge.yaml
│   │   └── ground_truth_bridge.yaml
│   ├── cartographer/
│   │   └── turtlebot3_lds_2d.lua
│   ├── exploration/
│   │   └── explore.yaml
│   ├── gazebo/
│   │   └── gui.config
│   ├── localization/
│   │   └── ekf.yaml
│   ├── nav2/
│   │   ├── burger_maze.yaml
│   │   └── burger_maze_dynamic.yaml
│   └── slam_toolbox/
│       └── mapper_maze.yaml
├── launch/
│   ├── ekf.launch.py
│   ├── exploration.launch.py
│   ├── localization_dynamic.launch.py
│   ├── mapping.launch.py
│   ├── mapping_slam_toolbox.launch.py
│   ├── navigation.launch.py
│   └── simulation.launch.py
├── maps/
│   ├── exploration/
│   │   ├── maze_ekf.pgm
│   │   └── maze_ekf.yaml
│   └── ground_truth/
│       ├── maze_gt.pcd
│       ├── maze_gt.pgm
│       └── maze_gt.yaml
├── models/
│   ├── gates/
│   │   ├── model.config
│   │   └── model.sdf
│   ├── maze/
│   │   ├── meshes/
│   │   ├── model.config
│   │   └── model.sdf
│   └── turtlebot3_burger/
│       └── model.sdf
├── package.xml
├── rviz/
│   ├── turtlebot3_conf2.rviz
│   └── turtlebot_explore.rviz
├── scripts/
│   ├── benchmark_route.py
│   ├── camera_controller.py
│   ├── cartographer_odometry.py
│   ├── ground_truth_odometry.py
│   ├── odom_covariance.py
│   └── random_gates.py
└── worlds/
    └── maze_world.sdf
```

### Configuration Files

| Directory | Description |
|---|---|
| `config/bridges/` | Configures communication between Gazebo Transport and ROS 2, including simulated sensors, velocity commands, and ground-truth data. |
| `config/cartographer/` | Contains the 2D LiDAR configuration for the alternative Cartographer SLAM implementation. |
| `config/exploration/` | Stores the Explore Lite parameters used for autonomous frontier exploration. |
| `config/gazebo/` | Contains the Gazebo graphical interface and camera configuration. |
| `config/localization/` | Contains the EKF configuration for sensor fusion. |
| `config/nav2/` | Provides separate Nav2 configurations for mapping and known-map navigation. |
| `config/slam_toolbox/` | Contains the SLAM Toolbox configuration for occupancy-grid mapping. |

The two Nav2 configurations are intentionally maintained separately:

- `burger_maze.yaml` is used for navigation during mapping and autonomous exploration.
- `burger_maze_dynamic.yaml` is used for goal-directed navigation with a previously generated map and potentially changing obstacles.

This separation allows navigation parameters to be tuned independently for each stage.

### Launch Files

| File | Purpose |
|---|---|
| `simulation.launch.py` | Starts the Gazebo maze simulation, inserts the TurtleBot3, and optionally enables gates, additional obstacles, and ground-truth features. |
| `ekf.launch.py` | Starts the EKF configured to fuse wheel-odometry information and IMU measurements. |
| `mapping_slam_toolbox.launch.py` | Starts SLAM Toolbox for building the occupancy grid. |
| `mapping.launch.py` | Alternative mapping launch, retained alongside the SLAM Toolbox implementation. |
| `navigation.launch.py` | Starts Nav2 for navigation during mapping and exploration. |
| `exploration.launch.py` | Starts Explore Lite for autonomous frontier exploration. |
| `localization_dynamic.launch.py` | Starts known-map localization using Map Server and AMCL. |

### Scripts

| File | Purpose |
|---|---|
| `odom_covariance.py` | Processes simulated wheel odometry and supplies the covariance information used by the EKF. |
| `ground_truth_odometry.py` | Provides ground-truth odometry data for experimental evaluation. |
| `cartographer_odometry.py` | Odometry-related utility for the alternative Cartographer workflow. |
| `random_gates.py` | Controls the moving gates and their opening and closing behavior. |
| `camera_controller.py` | Controls the Gazebo camera for monitoring the simulation. |
| `benchmark_route.py` | Utility associated with navigation route benchmarking and performance evaluation. |

The alternative Cartographer workflow and benchmarking utility are retained in the package. The commands documented below describe the currently used SLAM Toolbox and Nav2 workflows.

### Models and Maps

The `models/` directory contains the SDF representations of the TurtleBot3 Burger, the maze, and the gates.

The `worlds/maze_world.sdf` file defines the Gazebo simulation world.

The simulation launch can scale the maze geometry at runtime and optionally insert entrance and exit closure walls.

The `maps/` directory separates two different kinds of occupancy grids:

| File | Description |
|---|---|
| `maps/ground_truth/maze_gt.yaml` | Metadata for the reference occupancy map generated from the simulated environment. |
| `maps/ground_truth/maze_gt.pgm` | Reference occupancy-grid image. |
| `maps/ground_truth/maze_gt.pcd` | Point-cloud representation generated during reference-map creation. |
| `maps/exploration/maze_ekf.yaml` | Metadata for the occupancy grid generated by SLAM. |
| `maps/exploration/maze_ekf.pgm` | Occupancy-grid image reconstructed by the robot. |

The ground-truth map is used for controlled experiments and quantitative evaluation. It is not required as input during autonomous SLAM mapping.

---

# State Estimation and Coordinate Frames

Both operational stages use an Extended Kalman Filter implemented with `robot_localization`.

The estimation pipeline is:

```text
Gazebo wheel odometry
        |
        v
    /odom_raw
        |
        v
odom_covariance.py
        |
        v
    /odom_cov ------+
                    |
                    v
                   EKF <------ /imu
                    |
                    v
                  /odom
                    |
                    v
         odom -> base_footprint
```

The EKF combines wheel-odometry information with IMU angular-velocity measurements to provide a more stable estimate of the robot's local motion.

This is particularly important during rotations, where wheel slip and odometry errors can affect LiDAR alignment and localization.

The EKF publishes the local odometry transform:

```text
odom -> base_footprint
```

A separate global-localization component publishes:

```text
map -> odom
```

During Stage 1, this transform is provided by SLAM Toolbox.

During Stage 2, it is provided by AMCL.

The resulting TF structure is:

```text
map
 └── odom
      └── base_footprint
           └── base_link
                └── sensor frames
```

SLAM Toolbox and AMCL should not simultaneously publish `map -> odom` in the configurations described here.

Ground-truth odometry can be enabled separately for evaluation. It is not used as an EKF input in the documented estimation pipeline.

---

# Workspace Build and Preparation

The following instructions assume that the ROS 2 Jazzy Docker container is already running and that the user has entered it.

Build the workspace before the initial execution or after relevant package changes:

```bash
cd /ros2_ws

colcon build --symlink-install

source install/setup.bash
```

Each new terminal inside the container should load the ROS 2 environment and workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

**Important:** The numbered terminals below are separate terminals. Long-running launch processes should remain active while subsequent components are started.

A Docker image rebuild is not normally required for changes to the project's Python files, YAML configurations, or launch files. However, the ROS 2 workspace may need to be rebuilt depending on how package files are installed.

---

# Stage 1 — Autonomous Mapping and Exploration

## Objective

The robot explores an initially unknown maze, detects walls using its 2D LiDAR, and generates a map using SLAM Toolbox.

The maze is static during this stage. Moving gates and additional obstacles are disabled.

The mapping pipeline consists of:

```text
Gazebo
   |
   +--> Wheel odometry + IMU
   |            |
   |            v
   |           EKF
   |            |
   |            v
   |       Local odometry
   |            |
   +--> LiDAR --+--> SLAM Toolbox
                       |
                       +--> /map
                       +--> map -> odom
                                |
                                v
                               Nav2
                                ^
                                |
                            Explore Lite
```

SLAM Toolbox performs mapping and global pose correction, while Nav2 is responsible for path planning and execution.

Explore Lite selects exploration goals based on frontiers between known and unknown regions.

## Launch Procedure

### Terminal 1 — Gazebo Simulation

```bash
ros2 launch turtlebot_maze_sim simulation.launch.py \
    scale:=1.5 \
    closed_maze:=true \
    gates:=false \
    obstacle:=false \
    ground_truth:=false \
    generate_map:=false
```

The parameters have the following meanings:

| Parameter | Description |
|---|---|
| `scale:=1.5` | Scales the horizontal maze geometry by a factor of 1.5. |
| `closed_maze:=true` | Includes the entrance and exit closure walls. |
| `gates:=false` | Disables the moving gates. |
| `obstacle:=false` | Disables the additional static obstacle. |
| `ground_truth:=false` | Disables ground-truth odometry publication. |
| `generate_map:=false` | Does not regenerate the reference occupancy map. |

### Terminal 2 — Odometry Covariance Processing

```bash
ros2 run turtlebot_maze_sim odom_covariance.py
```

The node receives raw odometry and publishes the processed odometry used by the EKF:

```text
/odom_raw -> /odom_cov
```

### Terminal 3 — Extended Kalman Filter

```bash
ros2 launch turtlebot_maze_sim ekf.launch.py
```

The EKF uses:

```text
config/localization/ekf.yaml
```

It produces filtered odometry and publishes the `odom -> base_footprint` transformation.

### Terminal 4 — SLAM Toolbox

```bash
ros2 launch turtlebot_maze_sim mapping_slam_toolbox.launch.py
```

The mapping configuration is stored in:

```text
config/slam_toolbox/mapper_maze.yaml
```

The current configuration uses scan matching and is adapted to the dimensions of the maze.

The map resolution is configured as:

```yaml
resolution: 0.03
```

This corresponds to a 3 cm occupancy-grid resolution.

### Terminal 5 — Nav2

```bash
ros2 launch turtlebot_maze_sim navigation.launch.py
```

This launch uses the mapping-stage navigation configuration:

```text
config/nav2/burger_maze.yaml
```

Nav2 receives the map generated by SLAM Toolbox and provides the planning and control capabilities required by exploration.

### Terminal 6 — Autonomous Exploration

```bash
ros2 launch turtlebot_maze_sim exploration.launch.py
```

Explore Lite identifies frontiers and sends navigation objectives to Nav2.

The robot explores the known/unknown boundaries until the exploration algorithm determines that no further suitable frontiers remain.

### Terminal 7 — RViz2

```bash
rviz2 -d src/turtlebot_maze_sim/rviz/turtlebot_explore.rviz
```

RViz allows the operator to observe the map being constructed, LiDAR observations, robot pose, TF frames, planned paths, and costmaps.

For map visualization, the recommended configuration is:

```text
Fixed Frame: map
Map Topic: /map
Durability: Transient Local
LaserScan Topic: /scan
```

RViz can also be started before Explore Lite for manual navigation tests.

## Saving the Generated Map

Once a satisfactory map has been constructed, save it using:

```bash
ros2 run nav2_map_server map_saver_cli \
    -f /ros2_ws/src/turtlebot_maze_sim/maps/exploration/maze_ekf \
    --ros-args -p use_sim_time:=true
```

This generates:

```text
maps/exploration/
├── maze_ekf.pgm
└── maze_ekf.yaml
```

The `.pgm` file contains the occupancy-grid image, while the `.yaml` file contains the map resolution, origin, and related metadata.

The resulting map can subsequently be loaded by Map Server and used with AMCL during Stage 2.

---

# Stage 2 — Known-Map Navigation

## Objective

During the second stage, the robot starts with a previously generated or reference map and navigates toward a goal selected by the operator.

Unlike Stage 1, SLAM Toolbox is not executed.

Instead, Map Server provides the existing map and AMCL estimates the robot's global pose by comparing LiDAR observations with the known environment.

Nav2 calculates and executes paths toward the requested goal.

The environment may contain moving gates and additional obstacles that are not represented in the static map.

The navigation pipeline is:

```text
Saved map
    |
    v
Map Server --------+
                   |
                   v
LiDAR -----------> AMCL <---- EKF odometry
                   |
                   v
              map -> odom
                   |
                   v
                  Nav2
                   |
                   v
             TurtleBot3
```

The current experimental workflow uses the reference ground-truth map to isolate and evaluate localization and navigation behavior.

The final integrated workflow is intended to use the occupancy grid generated during Stage 1.

## Launch Procedure

### Terminal 1 — Gazebo Simulation

```bash
ros2 launch turtlebot_maze_sim simulation.launch.py \
    scale:=1.5 \
    closed_maze:=true \
    gates:=false \
    obstacle:=false \
    ground_truth:=true \
    generate_map:=false
```

This configuration starts the known-map navigation scenario without additional gates or obstacles.

Ground-truth odometry is enabled for experimental evaluation. It is not used as the localization input.

The moving gates and the optional obstacle can be enabled independently when required.

### Terminal 2 — Odometry Covariance Processing

```bash
ros2 run turtlebot_maze_sim odom_covariance.py
```

### Terminal 3 — Extended Kalman Filter

```bash
ros2 launch turtlebot_maze_sim ekf.launch.py
```

The same odometry-processing and EKF configuration are reused from Stage 1.

### Terminal 4 — AMCL and Map Server

```bash
ros2 launch turtlebot_maze_sim localization_dynamic.launch.py \
    map:=/ros2_ws/src/turtlebot_maze_sim/maps/ground_truth/maze_gt.yaml \
    params_file:=/ros2_ws/src/turtlebot_maze_sim/config/nav2/burger_maze_dynamic.yaml \
    use_sim_time:=true \
    autostart:=true
```

This starts the localization components, including Map Server, AMCL, and the localization lifecycle manager.

`autostart:=true` requests automatic configuration and activation of the managed localization nodes.

The reference map is used for the current controlled simulation experiments.

To use the map constructed during Stage 1, the map argument can instead be changed to:

```bash
map:=/ros2_ws/src/turtlebot_maze_sim/maps/exploration/maze_ekf.yaml
```

When switching maps, the AMCL initial pose must also correspond to the coordinate system and actual starting location in the selected map.

### Terminal 5 — Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/ros2_ws/src/turtlebot_maze_sim/config/nav2/burger_maze_dynamic.yaml \
    use_sim_time:=true \
    autostart:=true \
    use_composition:=False
```

This starts the Nav2 navigation stack using the configuration dedicated to known-map navigation.

The navigation stack includes path planning, local control, costmaps, recovery behaviors, velocity smoothing, and collision monitoring.

The current configuration uses NavFn for global path planning and DWB for local trajectory control.

### Terminal 6 — RViz2

```bash
rviz2 -d src/turtlebot_maze_sim/rviz/turtlebot_explore.rviz
```

In RViz, the operator can send a navigation goal using the available Nav2 goal tool.

The robot then attempts to follow the planned path while monitoring the environment using its LiDAR.

## Dynamic Gates and Additional Obstacles

The simulation launch supports independent activation of moving gates and an additional static obstacle.

For example:

```bash
ros2 launch turtlebot_maze_sim simulation.launch.py \
    scale:=1.5 \
    closed_maze:=true \
    gates:=true \
    obstacle:=true \
    ground_truth:=true \
    generate_map:=false
```

The two options are independent:

| Gates | Obstacle | Scenario |
|---|---|---|
| Disabled | Disabled | Original maze. |
| Enabled | Disabled | Maze with moving gates. |
| Disabled | Enabled | Maze with an additional static obstacle. |
| Enabled | Enabled | Maze with both gates and obstacle. |

The optional obstacle is generated as a box with collision and visual geometry.

Its position, horizontal dimensions, and orientation can be configured using launch arguments:

```text
obstacle_x
obstacle_y
obstacle_yaw
obstacle_length
obstacle_width
obstacle_height
```

Horizontal position and dimensions are scaled according to the `scale` argument. Height remains unchanged.

The additional obstacle is inserted after reference-map generation, so it is not included in the original ground-truth occupancy grid.

This allows experiments where the robot must detect an unexpected obstruction and calculate an alternative route if one exists.

Moving-gate navigation, waiting decisions, and time-efficient route replanning remain subjects of further development and evaluation.

---

# Navigation and Localization Development

Several improvements have been incorporated during development to address localization drift and unstable navigation behavior.

### Odometry Fusion

The EKF was introduced to fuse wheel-odometry information with IMU angular velocity.

This improves local motion estimation, particularly during rotations.

### AMCL Configuration

AMCL parameters were adapted to the dimensions and resolution of the maze.

The objective is to improve alignment between the LiDAR observations and the known occupancy grid.

### SLAM Configuration

SLAM Toolbox was configured to use the improved odometry estimate, scan matching, and a 3 cm map resolution.

The scan-matching search parameters were adapted to the relatively small and repetitive geometry of the maze.

### Navigation Controller

DWB parameters were adjusted to improve trajectory following through narrow corridors and sharp turns.

Important parameters include linear and angular velocity limits, acceleration constraints, simulation horizon, and trajectory critics.

### Collision Monitoring

Collision Monitor was investigated after observing navigation oscillations and repeated failures near corners.

The predictive `FootprintApproach` behavior was disabled during controlled static-environment experiments because its removal improved navigation behavior.

This is an experimental configuration decision, not a general safety recommendation for physical robots.

Collision protection and stopping distances must be validated again before real-world deployment.

### Current Status

The simulation currently supports:

- A scalable maze with optional closure walls.
- Independently configurable moving gates and an additional obstacle.
- Simulated wheel odometry, IMU, and LiDAR.
- EKF-based odometry estimation.
- SLAM Toolbox mapping.
- Autonomous exploration using Explore Lite.
- Saved occupancy-grid maps.
- AMCL localization with a known map.
- Nav2 goal-directed navigation.
- Ground-truth data for experimental evaluation.

Initial tests have produced a coherent SLAM map and substantially improved navigation behavior.

Further tuning and quantitative evaluation are still required, especially for narrow turns, obstacle avoidance, gate interaction, and total navigation time.

The final objective remains to integrate both stages and deploy the resulting autonomy software on the physical TurtleBot3 through the existing ROS 2 and Zenoh infrastructure.

---

# Contributors

- Agahan Yuldashev
- Al Faiz
- Luis Antonio
- Ulrich Jordan
- Ikram

---

# License

This project is released under the BSD License.