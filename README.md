
# Steam Deck Rover - ROS2 & PX4 - ARK Jetson PAB Carrier and ARK V6X

## Overview
A work in progress using a Steam Deck to controll a rover with ROS2. More details to come.

Contact braden@arkelectron.com for questions

## Steam Deck Instructions
On our steam deck we've installed Ubuntu 20.04 using distrobox. More info on how to do that [here](https://www.gamingonlinux.com/2022/09/distrobox-can-open-up-the-steam-deck-to-a-whole-new-world/)

To enter distrobox run
```
distrobox-enter ubuntu-20.04
```
On the steam deck clone the package into your ROS2 workspace. Make sure you have the px4_msgs package as well.
Run
```
colcon build
source install/setup.bash
```

Now run the one launch file
```
ros2 launch ark_rover steam_deck.launch.py
```

## Jetson Instructions
On the Jetson clone the package into your ROS2 workspace. Make sure you have the px4_msgs package as well.
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



