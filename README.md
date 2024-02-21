# Diff Drive Robot Simulation in ROS2 Humble

## Installation

Install the dependencies

```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```
colcon build --symlink-install
```

## Test

source dependencies first

```
source install/setup.bash
ros2 launch finalproject display.launch.py
```
