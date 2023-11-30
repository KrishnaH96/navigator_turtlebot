# navigator_turtlebot

## Overview
This repository contains a ros2 package(C++) which helps turtlebot navigate given map without running into any obstacles.


## Dependencies
* ROS 2 Humble
* Ubuntu 22.04

## Build Instructions

### Clone the repo:
```
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

git clone https://github.com/KrishnaH96/navigator_turtlebot.git

cd ..

rosdep install -i --from-path src --rosdistro humble -y

colcon build 

```
### ROS Bag:

Follow below instruction

```
source /opt/ros/humble/setup.bash

cd ros2_ws

source install/local_setup.bash

ros2 launch navigate_turtlebot navigator_launch.py bag_record:=True

```
I have added the recorded bag file in the file rosbag2 inside the package, run below command to check the rosbag I generated:

```
source /opt/ros/humble/setup.bash

cd ros2_ws

source install/local_setup.bash

cd src/navigator_turtlebot/rosbag2_gazebo/

#Check the rosbag using below command.
ros2 bag info rosbag2_gazebo.db3

#Use below command to playback the contets of the bag
ros2 bag play rosbag2_gazebo.db3

```

