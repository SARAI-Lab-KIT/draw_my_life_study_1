# PixelBot Display

## Overview

This package allows to perform facial animations on PixelBot's LCD. 

**Keywords:** facial animation, human-robot interaction, Pygame

The pixelbot_display package has been tested under [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on Ubuntu 24.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/jazzy/index.html) (middleware for robotics).
- [Pygame](https://www.pygame.org/news) (Python game developping library) for the visual animation of the robot's emotions on its LCD.
    ```
	sudo apt install python3-pygame
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_display
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_display pixelbot_display_node
```

## Nodes

### pixelbot_display_node

Display facial animations on the robot's LCD. 

#### Subscribed Topics

* **`/display_robot_state`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The robot state to be displayed on the robot's LCD.

## Troubleshooting

In the Draw My Life activity, the drawing application is supposed to run on the primary display, while the eyes animations, handled my this node, are ran on the secondary display. Make sure that your system settings correctly set the display order (tablet: primary display, robot's LCD: secondary display).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
