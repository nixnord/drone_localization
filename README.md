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
colcon build
```

3. Set environment variable for Gazebo
```bash
export GZ_SIM_RESOURCE_PATH=complete-path/drone_localization/models
```

<h2>Usage</h2>

<b>Step 1</b> Run Gazebo simulation

```bash
gz sim worlds/simworld.sdf
```
<hr>
<b>Step 2</b> Run Gazebo - ROS2 bridge

```bash
ros2 run ros_gz_bridge parameter_bridge world/trilateration_world/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V
```

This will bridge Gazebo and ROS2 topics essential for nodes to run.
<hr>
<b>Step 3</b> Source the package and nodes

```bash
source install/setup.bash
```
<hr>
<b>Step 4</b> Run the distance calculator node (RangeFinder)

```bash
ros2 run trilateration_nodes rangefinder
```

This node will calculate distance of drone from each of the three ground stations
<hr>
<b>Step 5</b> Run the trilateration node

```bash
ros2 run trilateration_nodes trilateration_node
```

This will estimate the drone coordinates using trilateration
<hr>
<b>Step 6</b> Run Rviz2 (For position/trajectory visualization)

```bash
rviz2 -d config.rviz
```
<hr>
<b>Step 7</b> Run visualizer node (Feeds data to Rviz)

```bash
ros2 run trilateration_nodes visualizernode
```
<hr>
All nodes must be running parallely for results.
The installation/deploying process is tedious as of now so I will be uploading the launch files in the next commit which will simplify it.


