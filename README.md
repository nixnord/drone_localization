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

3. Set environment variable for Gazebo
```bash
export GZ_SIM_RESOURCE_PATH=complete-path/drone_localization/models
```

<b>OPTIONAL:</b> Check if you have the `ros_gz_bridge` package installed.
```bash
ros2 pkg list | grep ros_gz_bridge
```
If you don't get any output, install it referring from the [documentation](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)

<h2>Usage</h2>

<b>Step 1</b> Run Gazebo simulation

```bash
gz sim worlds/simworld.sdf
```
<hr>
<b>Step 2</b> Source the package

```bash
source install/setup.bash
```
<hr>
<b>Step 3</b> Launch ROS2 Nodes

```bash
ros2 launch trilateration_nodes nodes.launch.py
```
<p>The above command will also launch Rviz2 which will show the realtime estimates of the position of the drone.</p>
<hr>


