
# Steam Deck Rover - ROS2 & PX4 - ARK Jetson PAB Carrier and ARK V6X

## Overview
A work in progress using a Steam Deck to controll a rover with ROS2. More details to come.

Contact braden@arkelectron.com for questions

## Steam Deck Prerequisites
Distrobox Ubuntu 20.04 container with ROS2 Galactic
Distrobox Ubuntu 22.04 container with ROS2 Humble

On our steam deck we've installed distrobox and set up two different containers. More info on how to do that [here](https://www.gamingonlinux.com/2022/09/distrobox-can-open-up-the-steam-deck-to-a-whole-new-world/)

To enter distrobox run
```
distrobox-enter ubuntu-20.04
```
On the steam deck clone the package into your ROS2 workspace. You also need the [px4_msgs](https://github.com/PX4/px4_msgs) package.
```
colcon build
source install/setup.bash
```

Now run the one launch file
```
ros2 launch ark_rover steam_deck.launch.py
```

In a new console open a new distrobox container
```
distrobox-enter ubuntu-22.04
source /opt/ros/humble/setup.bash
```
Install and run Foxglove Bridge
```
sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Go to this [link](https://studio.foxglove.dev/) to open Foxglove and open a Foxglove Websocket

## Jetson Instructions
On the Jetson clone the package into your ROS2 workspace. You also need the [px4_msgs](https://github.com/PX4/px4_msgs) package and the [depthai-ros](https://github.com/luxonis/depthai-ros.git) package. It's very important that you are on the Galactic branch.
Run
```
colcon build
source install/setup.bash
```

Now run the one launch file
```
ros2 launch ark_rover jetson.launch.py
```

## Control Instructions
- Arm/Disarm - A
- Throttle - Left Stick Y
- Steering - Right Stick X
- Low Gear - D-Pad Down (Default)
- High Gear - D-Pad Up
- Unlocked Differential - D-Pad Left (Default)
- Locked Differential - D-Pad Right



