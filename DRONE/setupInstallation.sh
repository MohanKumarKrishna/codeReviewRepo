#!/bin/bash

# ==============================================================================
# AUTOMATED SETUP SCRIPT: ROS 2 + ArduPilot + Gazebo Harmonic + Custom Bridge
# ==============================================================================

# Stop on any error
set -e

# Define colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log() {
    echo -e "${BLUE}[SETUP] $1${NC}"
}

success() {
    echo -e "${GREEN}[SUCCESS] $1${NC}"
}

# Ensure script is NOT run as root (it messes up home directory paths)
if [ "$EUID" -eq 0 ]; then
  echo "Please run this script as your normal user (not root/sudo)."
  echo "You will be asked for sudo password when needed."
  exit 1
fi

# Ask for sudo upfront to refresh timestamp
log "Requesting sudo permissions..."
sudo -v

# Keep sudo alive
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# ==============================================================================
# 1. System Locale & Basic Tools
# ==============================================================================
log "Configuring Locale and installing base tools..."

# Install locales
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install base dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    software-properties-common \
    build-essential \
    git \
    curl \
    wget \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    mesa-utils \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    at-spi2-core

# ==============================================================================
# 2. ROS 2 Humble Installation
# ==============================================================================
log "Installing ROS 2 Humble..."

# Add Universe repo
sudo add-apt-repository -y universe

# Add GPG Key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add Repo
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/0

# Install
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep

# Init Rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Add to bashrc if not present
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Source strictly for this script execution
source /opt/ros/humble/setup.bash

# ==============================================================================
# 3. MAVROS & GeographicLib
# ==============================================================================
log "Installing MAVROS and Geographic Datasets..."

sudo apt install -y ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geographiclib_datasets.sh
chmod +x /tmp/install_geographiclib_datasets.sh
sudo /tmp/install_geographiclib_datasets.sh

# ==============================================================================
# 4. ArduPilot Setup
# ==============================================================================
log "Setting up ArduPilot..."

cd ~
if [ ! -d "ardupilot" ]; then
    git clone https://github.com/ArduPilot/ardupilot.git
fi
cd ardupilot
git checkout Copter-4.5.7
git submodule update --init --recursive

# Create fixed prereq installer
sed 's|sudo usermod -a -G dialout.*|true|' Tools/environment_install/install-prereqs-ubuntu.sh > Tools/environment_install/install-prereqs-fixed.sh
chmod +x Tools/environment_install/install-prereqs-fixed.sh

# Run fixed installer (skip sudo prompt inside script)
./Tools/environment_install/install-prereqs-fixed.sh -y

# Fix user group manually
sudo usermod -a -G dialout "$USER"

# Create clean env file
cat <<'EOF' > ~/.ardupilot_env
# ArduPilot environment
source "$HOME/ardupilot/Tools/completion/completion.bash"
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"
export PATH="/usr/lib/ccache:$PATH"
EOF

# Link env to bashrc
if ! grep -q "source ~/.ardupilot_env" ~/.bashrc; then
    echo 'source ~/.ardupilot_env' >> ~/.bashrc
fi

# Python setuptools fix
pip3 install --user "setuptools<80" pymavlink MAVProxy

# Add local bin to path for this session
export PATH="$HOME/.local/bin:$PATH"
if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' ~/.bashrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
fi

# ==============================================================================
# 5. SITL Hardware Interface
# ==============================================================================
log "Building ROS 2 ArduPilot SITL Interface..."

cd ~
if [ ! -d "ros2-ardupilot-sitl-hardware" ]; then
    git clone https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware.git
fi
cd ros2-ardupilot-sitl-hardware

# Fix python versions for build
python3 -m pip install --user "setuptools<69" "packaging>=21,<24"

rm -rf build install log
colcon build
source install/setup.bash

# ==============================================================================
# 6. Gazebo Harmonic & Plugins
# ==============================================================================
log "Installing Gazebo Harmonic and ArduPilot Plugin..."

# Keys and Repos
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/0

sudo apt update
sudo apt install -y gz-harmonic gazebo libgazebo-dev
sudo apt install -y libgz-sim8-dev rapidjson-dev \
    libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

# ArduPilot Gazebo Plugin
mkdir -p ~/simtofly_ws
cd ~/simtofly_ws
if [ ! -d "ardupilot_gazebo" ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git
fi
cd ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4

# Export Gazebo Vars
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/simtofly_ws/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

# ==============================================================================
# 7. Custom ROS-Gazebo Bridge
# ==============================================================================
log "Setting up Custom IMU/Image Bridge..."

# Install Deps
sudo apt install -y libgz-transport12-dev libgz-msgs9-dev \
    ros-humble-rclcpp ros-humble-sensor-msgs

mkdir -p ~/gz_ros_min_bridge/src
cd ~/gz_ros_min_bridge/src

# Create Pkg
if [ ! -d "gz_ros_imu_bridge" ]; then
    ros2 pkg create --build-type ament_cmake gz_ros_imu_bridge --dependencies rclcpp sensor_msgs
fi

# Write imu_bridge.cpp
cat << 'EOF' > ~/gz_ros_min_bridge/src/gz_ros_imu_bridge/src/imu_bridge.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuBridge : public rclcpp::Node
{
public:
  ImuBridge() : Node("gz_imu_bridge")
  {
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/gazebo/imu", 10);
    std::string topic = "/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu";
    gz_node_.Subscribe(topic, &ImuBridge::cb, this);
    RCLCPP_INFO(get_logger(), "IMU bridge running");
  }

private:
  void cb(const gz::msgs::IMU & msg)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = "imu_link";

    tf2::Quaternion q(msg.orientation().x(), msg.orientation().y(), msg.orientation().z(), msg.orientation().w());
    q.normalize();
    imu.orientation.x = q.x(); imu.orientation.y = q.y(); imu.orientation.z = q.z(); imu.orientation.w = q.w();

    tf2::Vector3 w_gz(msg.angular_velocity().x(), msg.angular_velocity().y(), msg.angular_velocity().z());
    tf2::Vector3 w_ros = tf2::quatRotate(q, w_gz);
    imu.angular_velocity.x = w_ros.x(); imu.angular_velocity.y = w_ros.y(); imu.angular_velocity.z = w_ros.z();

    tf2::Vector3 a_gz(msg.linear_acceleration().x(), msg.linear_acceleration().y(), msg.linear_acceleration().z());
    tf2::Vector3 a_rot = tf2::quatRotate(q, a_gz);
    tf2::Vector3 g(0, 0, 9.80665);
    tf2::Vector3 a_ros = a_rot - g;
    imu.linear_acceleration.x = a_ros.x(); imu.linear_acceleration.y = a_ros.y(); imu.linear_acceleration.z = a_ros.z();

    imu.orientation_covariance[0] = -1;
    imu.angular_velocity_covariance[0] = 0.01;
    imu.linear_acceleration_covariance[0] = 0.1;

    pub_->publish(imu);
  }
  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBridge>());
  rclcpp::shutdown();
  return 0;
}
EOF

# Write image_bridge.cpp
cat << 'EOF' > ~/gz_ros_min_bridge/src/gz_ros_imu_bridge/src/image_bridge.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

class ImageBridge : public rclcpp::Node
{
public:
  ImageBridge() : Node("gz_image_bridge")
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>("/gazebo/camera/image", 10);
    std::string topic = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image";
    gz_node_.Subscribe(topic, &ImageBridge::cb, this);
    RCLCPP_INFO(get_logger(), "Image bridge running");
  }

private:
  void cb(const gz::msgs::Image & msg)
  {
    sensor_msgs::msg::Image img;
    img.header.stamp = now();
    img.header.frame_id = "pitch_link";
    img.height = msg.height();
    img.width  = msg.width();
    img.step   = msg.step();
    img.encoding = "rgb8";
    img.is_bigendian = false;
    const std::string & data = msg.data();
    img.data.assign(data.begin(), data.end());
    pub_->publish(img);
  }
  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageBridge>());
  rclcpp::shutdown();
  return 0;
}
EOF

# Write CMakeLists.txt
cat << 'EOF' > ~/gz_ros_min_bridge/src/gz_ros_imu_bridge/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(gz_ros_imu_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gz-transport12 CONFIG REQUIRED)
find_package(gz-msgs9 CONFIG REQUIRED)

add_executable(imu_bridge src/imu_bridge.cpp)
add_executable(image_bridge src/image_bridge.cpp)

ament_target_dependencies(imu_bridge rclcpp sensor_msgs tf2)
ament_target_dependencies(image_bridge rclcpp sensor_msgs)

target_link_libraries(imu_bridge gz-transport12::gz-transport12 gz-msgs9::gz-msgs9)
target_link_libraries(image_bridge gz-transport12::gz-transport12 gz-msgs9::gz-msgs9)

install(TARGETS imu_bridge image_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
EOF

# Build Bridge
log "Building Bridge..."
cd ~/gz_ros_min_bridge
rm -rf build install log
colcon build --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==============================================================================
# 8. Completion
# ==============================================================================
success "All installation steps completed!"
echo -e "${BLUE}IMPORTANT NEXT STEPS:${NC}"
echo "1. You MUST log out and log back in for the 'dialout' group (Serial permissions) to take effect."
echo "2. After logging back in, you can run your bridge using:"
echo "   source ~/gz_ros_min_bridge/install/setup.bash"
echo "   ros2 run gz_ros_imu_bridge imu_bridge & ros2 run gz_ros_imu_bridge image_bridge"
echo ""