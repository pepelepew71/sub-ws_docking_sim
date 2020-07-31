# sub-ws_docking_sim

## 1. Launch

### Docking Simulation

```bash
roslaunch docking_sim docking.launch
```

### Controlling Simulation

```bash
roslaunch docking_sim control.launch
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
```
