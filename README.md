# sub-ws_docking_sim

## 1. Launch

### Docking Simulation

Modify the parameter in obstacle_detector

```xml
    <!-- <param name="frame_id"             value="$(arg robot_name)/map"/> -->
    <!-- <remap from="scan" to="/$(arg robot_name)/scan_merged"/> -->
    <param name="frame_id" value="base_link"/>
    <remap from="scan" to="/base_link_scan"/>
```

```bash
roslaunch docking_sim dock.launch
```

### Linked Simulation

```bash
roslaunch docking_sim link.launch
```

## 2. Change World

Modify the argument world_name of the launch file in folder docking_gazebo/launch

```xml
    <!-- # gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find docking_gazebo)/worlds/empty.world" />
        <!-- <arg name="world_name" value="$(find docking_gazebo)/worlds/cloister.world" /> -->
        <!-- <arg name="world_name" value="$(find docking_gazebo)/worlds/shelf.world" /> -->
        <!-- <arg name="world_name" value="$(find docking_gazebo)/worlds/playpen-3.world" /> -->
        <arg name="paused" value="false"/>

```

## 3. Dependency

```bash
sudo apt install libarmadillo-dev

sudo apt install ros-kinetic-dwa-local-planner
sudo apt install ros-kinetic-rviz-imu-plugin
sudo apt install ros-kinetic-teb-local-planner
sudo apt install ros-kinetic-timed-roslaunch
```
