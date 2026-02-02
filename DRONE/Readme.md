# ROS 2 + ArduPilot + Gazebo Harmonic + Custom ROS–Gazebo Bridge

This repository provides a **fully automated setup** for a complete UAV simulation and integration stack using:

- **ROS 2 Humble**
- **ArduPilot (Copter 4.5.7)**
- **Gazebo Harmonic**
- **MAVROS**
- **Custom minimal ROS ↔ Gazebo IMU & Camera bridge**

The setup is designed for **Ubuntu 22.04 (Jammy)** and installs everything in a **reproducible, clean, non-root-safe** manner.

---

## 📦 What This Setup Provides

- ROS 2 Humble desktop environment
- ArduPilot SITL with MAVLink tooling
- Gazebo Harmonic simulator
- ArduPilot Gazebo plugins
- ROS 2 SITL hardware interface
- **Custom C++ bridge**:
  - Gazebo → ROS 2 IMU (`sensor_msgs/Imu`)
  - Gazebo → ROS 2 Camera (`sensor_msgs/Image`)
- Correct frame handling, gravity compensation, and covariance settings
- One-command automated installation

---

## 🧰 System Requirements

| Item | Requirement |
|----|----|
| OS | Ubuntu 22.04 LTS |
| Architecture | amd64 |
| ROS | ROS 2 Humble |
| User | **Non-root user with sudo access** |
| Internet | Required |

> ⚠️ **Do NOT run the script as root.**  
> The script exits automatically if run using `sudo`.

---



# 🐳 ArduPilot + ROS 2 + Gazebo (Docker Setup)

This repository documents a **Docker-based workflow** for running **ArduPilot SITL** with **Gazebo** and **ROS 2 (Humble)**, including GUI support, GPU acceleration, MAVROS, and Gazebo–ROS bridges.

---

## 📋 Prerequisites

- Linux host (tested on Ubuntu 20.04 / 22.04)
- Docker + Docker Compose
- NVIDIA GPU + drivers (optional, for GPU acceleration)
- NVIDIA Container Toolkit (`nvidia-docker2`)
- X11 installed on host

---

## 🖼️ Enable GUI Access (Host)

Allow Docker containers to access the X server:

```bash
xhost +local:docker
```

---

## 📦 Docker Basics

### Exec into the first running container
```bash
docker exec -it $(docker ps -q | head -n 1) /bin/bash
```

### Run ArduPilot base container (GUI + Host Networking)
```bash
docker run -it \
  --name ardupilot_ros \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --net=host \
  --privileged \
  ubuntu-ardupilot-base:22.04
```

### List all containers
```bash
docker ps -a
```

### Start an existing container interactively
```bash
docker start -ai ardupilot_ros
```

---

## 💾 Convert Container to Image

Save the configured container as a reusable image:

```bash
docker commit ardupilot_ros ardupilot_ros_image:latest
```

---

## 🔄 Restart / Access Container

```bash
docker restart ardupilot_ros
docker exec -it ardupilot_ros /bin/bash
```

---

## 🎮 Run Container with GPU + GUI Support

```bash
docker run -it \
  --name ardupilot_ros \
  --gpus all \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  ardupilot_ros_image:latest
```

---

## 🌍 Gazebo Simulation

### Launch Gazebo (verbose, real-time)
```bash
gz sim -v4 -r iris_runway.sdf
```

### Start ArduCopter SITL
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

---

## 🤖 ROS 2 Setup

### Source ROS 2 Humble
```bash
source /opt/ros/humble/setup.bash
```

### Start MAVROS (direct command — more reliable)
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://:14550@" \
  -p target_system_id:=1 \
  -p target_component_id:=1
```

---

## 🔗 Gazebo ↔ ROS 2 Bridges

### Source bridge workspace
```bash
source ~/gz_ros_min_bridge/install/setup.bash
```

### IMU Bridge
```bash
ros2 run gz_ros_imu_bridge imu_bridge
```

### Image Bridge
```bash
ros2 run gz_ros_imu_bridge image_bridge
```

### Run Both (background)
```bash
ros2 run gz_ros_imu_bridge imu_bridge & \
ros2 run gz_ros_imu_bridge image_bridge
```

---

## 📡 ROS 2 Topic Utilities

### List Gazebo-related topics
```bash
ros2 topic list | grep gazebo
```

### Echo IMU data
```bash
ros2 topic echo /gazebo/imu
```

### Inspect message structure
```bash
ros2 interface show <topic_message_type>
```

---

## 🖼️ Image Visualization

```bash
ros2 run rqt_image_view rqt_image_view
```

---

## 🚁 ArduPilot Troubleshooting (Simulation Only)

### Disable GPS
```bash
param set SIM_GPS_DISABLE 0
```

### Set Home Position
```bash
wp sethome
```

### Disable Arming Checks (⚠️ SITL only)
```bash
param set ARMING_CHECK 0
```

### EKF Reset
```bash
param set EK3_ENABLE 0
param set EK3_ENABLE 1
reboot
```

---

## 🧩 Shell Environment

```bash
source ~/.bashrc
source ~/.profile
```

---

## ✅ Recommended Startup Order

1. Enable X11 access (`xhost +local:docker`)
2. Start Docker container
3. Launch Gazebo
4. Start ArduPilot SITL
5. Source ROS 2
6. Start MAVROS
7. Start Gazebo–ROS bridges
8. Inspect topics / visualize data

---

## ⚠️ Notes

- All safety checks disabled commands are **for simulation only**
- Host networking is required for MAVROS + SITL UDP communication
- GPU flags require NVIDIA Container Toolkit

---

## 📌 TODO / Extensions

- Add Docker Compose setup
- Add launch scripts
- Add multi-vehicle support
- Add PX4 comparison

---

🚀 Happy flying!

