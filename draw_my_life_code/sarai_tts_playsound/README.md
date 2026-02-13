# Sarai Text to Speech - Playsound

## Overview

This package allows any robot to speak. 

**Keywords:**  human-robot interaction, audio communication, voice synthesis

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

**Author: Romain Maure<br />
Affiliation: [SARAI Lab, KIT](https://sarai.iar.kit.edu/)<br />
Maintainer: Romain Maure, romain.maure@kit.edu**

The sarai_tts_playsound package has been tested under [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on Ubuntu 24.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/jazzy/index.html) (middleware for robotics).
- [Piper](https://github.com/OHF-Voice/piper1-gpl): text to speech library for Python.
    ```
	sudo pip3 install piper-tts
    ```    


#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select sarai_tts_playsound
    ```

## Usage

You can run the main node with:
```
ros2 run sarai_tts_playsound sarai_tts_playsound_node
```

## Nodes

### sarai_tts_playsound_node

Allows any robot to speak.

#### Subscribed Topics

* **`/tts_input`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text to be said by the TTS engine.


#### Published Topics

* **`/display_robot_state`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The robot state to be displayed on the robot's LCD.

* **`/recognizer_is_active`** ([std_msgs/Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))

	The state of the speech recognition engine (activated/deactivated).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/SARAI-Lab-KIT/draw_my_life_study_1/issues).
