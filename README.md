# drone_localization

GPS-denied drone localization using LoRa trilateration, Kalman Filter fusion, and autonomous fire detection — built on PX4 SITL, ROS2 Humble, and Gazebo Harmonic.


## System Architecture

```
Gazebo Harmonic (gz_x500_mono_cam)
        │
        ├── /fmu/out/sensor_combined         → Kalman Filter (IMU accel, 250 Hz)
        ├── /fmu/out/vehicle_attitude        → Attitude quaternion (body→ENU rotation)
        ├── /fmu/out/vehicle_local_position  → Ground truth (EKF2, for comparison)
        │
        ├── Simulated LoRa ground stations
        │     └── drone_rangefinder.py       → /lora/range
        │           └── drone_trilateration.py → /drone/estimatedpose (ENU x,y)
        │                 └── visualizer.py  → /drone/filteredpose (Kalman Filter refined)
        │
        └── Camera sensor
              └── ros_gz_bridge             → /drone/camera/image_raw
                    └── fire_detection.py → POST /infer (FastAPI)
                          └── /drone/fire_detection (JSON result)
```

---

## Prerequisites

These must be installed manually before running `setup.sh`.

### 1. ROS2 Humble

Follow the [official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

```bash
# Verify
ros2 --version
```

### 2. Gazebo Harmonic + ROS2 Bridge

```bash
sudo apt install ros-humble-ros-gzharmonic
```

### 3. PX4-Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
```

> ⚠️ This project was developed against **PX4 v1.17.0-alpha1**. Using a different version may cause `px4_msgs` message definition mismatches.

### 4. GPU Driver (for fire detection inference)

Verify your NVIDIA driver is installed and the GPU is visible:

```bash
nvidia-smi
```

Driver version 525.60+ is required for CUDA 12.x support. TensorFlow GPU wheels are installed automatically by `setup.sh`.


## Installation

```bash
git clone https://github.com/nixnord/drone_localization.git
cd drone_localization

chmod +x setup.sh
./setup.sh

source ~/.bashrc
```

`setup.sh` will automatically:
- Clone and build `px4_msgs` into `~/px4_ros_ws`
- Build and install `MicroXRCEAgent`
- Install all ROS2 apt packages (`cv_bridge`, `rviz2`, etc.)
- Install all Python pip packages from `requirements.txt`
- Build the `trilateration_nodes` package


## Gazebo World and Fire Model Setup

The forest fire world and fire model must be copied into PX4's simulation directory:

```bash
cp forest_fire.sdf \
   ~/PX4-Autopilot/Tools/simulation/gz/worlds/

cp -r src/trilateration_nodes/models/firemodel \
   ~/PX4-Autopilot/Tools/simulation/gz/models/
```

## Running the Simulation

Open 5 terminals and run each command in order:

```bash
MicroXRCEAgent udp4 -p 8888

PX4_GZ_WORLD=forest_fire make px4_sitl gz_x500_mono_cam

ros2 run ros_gz_bridge parameter_bridge \
  /world/forest_fire/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args \
  -r /world/forest_fire/model/x500_mono_cam_0/link/camera_link/sensor/camera/image:=/drone/camera/image_raw

cd src/trilateration_nodes
uvicorn fire_server:app --host 0.0.0.0 --port 8000

ros2 launch trilateration_nodes simulation.launch.py
```

Topics to add in RViz2:

| Display Type | Topic | Description |
|---|---|---|
| Path | `/drone/estpath` | Raw LoRa trilateration path |
| Path | `/drone/filteredpath` | Kalman filter refined path |
| Path | `/drone/realpath` | PX4 EKF2 ground truth path |
| PoseStamped | `/drone/filteredpose` | Current KF position |
| Image | `/drone/camera/image_raw` | Live camera feed |


## Fire Detection Results

Fire detection results are published to `/drone/fire_detection` as a JSON string:

```bash
ros2 topic echo /drone/fire_detection
```

Example output:
```json
{
  "timestamp": 1234567890123456789,
  "fire_detected": true,
  "fire_pixel_count": 47,
  "fire_coverage_pct": 4.59,
  "confidence": 0.8231,
  "mask_file": "abc123.png"
}
```


## Ground Station Layout

Three LoRa ground stations placed at the vertices of an equilateral triangle (side 60m) with the drone at the centroid:

```
         GS1 (0, 34.64)
              △
             / \
            /   \
           / 🚁  \    ← drone spawns at (0, 0)
          /       \
GS2 (-30,-17.32)──GS3 (30,-17.32)
```

Circumradius (drone to each station): **34.64m**

## Dependency Versions

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble (LTS) |
| Gazebo | Harmonic |
| PX4-Autopilot | v1.14 |
| Python | 3.10 |
| TensorFlow | 2.20.0 |
| CUDA | 12.x |
| cuDNN | 8.9+ |

## License

Apache v2.0
