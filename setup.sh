#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
error()   { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

info "Checking prerequisites..."

if ! command -v ros2 &>/dev/null; then
    error "ROS2 Humble not found. Install it first: https://docs.ros.org/en/humble/Installation.html"
fi

if ! command -v gz &>/dev/null; then
    error "Gazebo Harmonic not found. Install via: sudo apt install ros-humble-ros-gzharmonic"
fi

if [ ! -d "$HOME/PX4-Autopilot" ]; then
    error "PX4-Autopilot not found at ~/PX4-Autopilot. Clone it first:
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
    bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh"
fi

info "All prerequisites found."

info "[1/5] Setting up px4_msgs workspace..."

PX4_ROS_WS="$HOME/px4_ros_ws"

if [ ! -d "$PX4_ROS_WS" ]; then
    info "Cloning px4_msgs..."
    mkdir -p "$PX4_ROS_WS/src"
    cd "$PX4_ROS_WS/src"
    git clone https://github.com/PX4/px4_msgs.git
    cd "$PX4_ROS_WS"
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    info "px4_msgs built successfully."
else
    warn "px4_ros_ws already exists at $PX4_ROS_WS — skipping clone."
fi

if ! grep -q "px4_ros_ws" ~/.bashrc; then
    echo "source $PX4_ROS_WS/install/setup.bash" >> ~/.bashrc
    info "Added px4_ros_ws to ~/.bashrc"
fi


info "[2/5] Setting up MicroXRCE-DDS Agent..."

if ! command -v MicroXRCEAgent &>/dev/null; then
    cd /tmp
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/
    info "MicroXRCEAgent installed successfully."
else
    warn "MicroXRCEAgent already installed — skipping."
fi

info "[3/5] Installing ROS2 apt packages..."

sudo apt update -qq
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-tools \
    ros-humble-ros-gzharmonic \
    ros-humble-rviz2 \
    python3-colcon-common-extensions

info "ROS2 apt packages installed."

info "[4/5] Installing Python pip packages..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
    pip install -r "$SCRIPT_DIR/requirements.txt"
else
    pip install numpy scipy requests opencv-python fastapi uvicorn tensorflow
fi

info "Python packages installed."

info "[5/5] Building trilateration_nodes package..."

WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ ! -f "$WS_ROOT/src/trilateration_nodes/package.xml" ]; then
    WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

cd "$WS_ROOT"
source /opt/ros/humble/setup.bash
source "$PX4_ROS_WS/install/setup.bash"
colcon build --packages-select trilateration_nodes --symlink-install

WS_INSTALL="$WS_ROOT/install/setup.bash"
if ! grep -q "$WS_INSTALL" ~/.bashrc; then
    echo "source $WS_INSTALL" >> ~/.bashrc
    info "Added workspace to ~/.bashrc"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Setup complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Run this to apply all changes to your current terminal:"
echo "  source ~/.bashrc"
echo ""
echo "Then start the simulation with:"
echo "  MicroXRCEAgent udp4 -p 8888"
echo "  PX4_GZ_WORLD=forest_fire make px4_sitl gz_x500_mono_cam"