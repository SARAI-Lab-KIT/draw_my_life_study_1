# Pixelbot Tablet

## Overview

This package allows running the drawing application on a tablet.

**Keywords:** drawing application

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

**Author: Romain Maure<br />
Affiliation: [SARAI Lab, KIT](https://sarai.iar.kit.edu/)<br />
Maintainer: Romain Maure, romain.maure@kit.edu**

The pixelbot_tablet package has been tested under [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on Ubuntu 24.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot  Operating System (ROS2)](https://docs.ros.org/en/jazzy/index.html) (middleware for robotics).
- [Pygame-ce](https://www.pygame.org/docs/) (Python game developping library) for developping the drawing application running on the tablet.
    ```
    sudo apt install python3-pygame-ce
    ```
- [OpenCV](https://pypi.org/project/opencv-python/) and [cv-bridge](https://pypi.org/project/cv-bridge/): Computer vision Python library.
    ```
	sudo pip3 install opencv-python
    sudo pip3 install cv-bridge
    ```  
- [evdev](https://pypi.org/project/evdev/) to obtain the data from the tablet and pen (pressure, etc).
    ```
    sudo pip3 install evdev
    ``` 

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_tablet
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_tablet draw_node
```

## Nodes

### draw_node

This package allows running the drawing application on a tablet.

#### Published Topics

* **`/drawings`** ([sensor_msgs/Image](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html))

	The latest version of the user drawing, to be processed by the VLM.

* **`/tts_input`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text to be said by the TTS engine.

* **`/display_robot_state`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The robot state to be displayed on the robot's LCD.

* **`/child_session_folder_path`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The path of the user session, for data saving purposes.

* **`/reset_drawing_session`** ([std_msgs/Empty](https://docs.ros2.org/foxy/api/std_msgs/msg/Empty.html))

	The reset signal when a Draw My Life activity is finished.

* **`/recognizer_is_active`** ([std_msgs/Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))

	The state of the speech recognition engine (activated/deactivated).

#### Parameters

* **`pressure_recording`** (string)

	Whether pressure data should be recorded. Enabled by default. If the drawing application is ran on a normal monitor, please disable this parameter.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/pixelbot/pixelbot_tablet/-/issues).