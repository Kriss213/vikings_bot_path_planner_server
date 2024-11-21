# vikings_bot_path_planner_server

<hr>

### Description
This package contains `vikings_bot` project path planner related files. It uses path planning methods from `ROS2 NAV2` stack.

<hr>

## Installation

Download source and install dependencies:
```
cd <path/to/your/ros_ws>
git clone git@github.com:Hercogs/vikings_bot_path_planner_server.git src/vikings_bot_path_planner_server
rosdep update
rosdep install --ignore-src --default-yes --from-path src
```

Build package:
```
colcon build
source install/setup.bash
```

<hr>

### Usage

To spawn path planner:
```ros2 launch vikings_bot_path_planner_server spawn_path_planner.launch.py```
#### Parameters:
- `vikings_bot_name`: namespace of robot - [vikings_bot_1 or vikings_bot_2] -> *string*, default *vikings_bot_1*
- `use_sim`: whether to use simulated or real robot -> *bool*, default *true*
- `use_lidar`: whether to use lidar for navigation -> *bool*, default *true*
- `use_depth_cam`: whether to use depth camera for navigation -> *bool*, default *false*

