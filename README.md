<h2>Drone Localization Simulation</h2>

<p>ROS2 - Gazebo / Rviz simulation of estimating the position of a drone real time using three point trilateration principle involving the use of LoRa modules</p>

<b>Developed and tested on ROS-2 Humble | Gazebo Harmonic</b>

<h2>Installation steps</h2>

1. Clone the repository
```bash
git clone https://github.com/nixnord/drone_localization.git
```

2. Build package
```bash
cd drone_localization
colcon build
```

<h2>Usage</h2>

1. Source the package

```bash
source install/setup.bash
```
<hr>

2. Launch ROS2 Nodes, Gazebo and Rviz2

```bash
ros2 launch trilateration_nodes nodes.launch.py
```
<p>The above command will also launch Gazebo, Rviz2 which will show the realtime estimates of the position of the drone.</p>
<hr>


