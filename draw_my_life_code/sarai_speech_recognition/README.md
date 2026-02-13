# SARAI Speech Recognition

## Overview

This package allows to recognize speech.

**Keywords:** speech recognition, audio

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

**Author: Romain Maure<br />
Affiliation: [SARAI Lab, KIT](https://sarai.iar.kit.edu/)<br />
Maintainer: Romain Maure, romain.maure@kit.edu**

The pixelbot_vlm package has been tested under [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on Ubuntu 24.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [SpeechRecognition](https://pypi.org/project/SpeechRecognition/) Python library for the speech recognition.
    ```
    sudo pip3 install SpeechRecognition
    ```
- [PyAudio](https://pypi.org/project/PyAudio/) Python library for audio related tasks.
    ```
    sudo pip3 install PyAudio
    ```
- [faster-whisper](https://pypi.org/project/faster-whisper/) to use Faster Whisper as speech recognizer.
    ```
    sudo pip3 install faster-whisper
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select sarai_speech_recognition
    ```

## Usage

You can run the main node with:
```
ros2 run sarai_speech_recognition sarai_speech_recognition_node
```

## Nodes

### sarai_speech_recognition_node

Allows to run the speech recognition.

#### Subscribed Topics

* **`/recognizer_is_active`** ([std_msgs/Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))

	The state of the speech recognition engine (activated/deactivated).

* **`/tts_input`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text to be said by the TTS engine.


#### Published Topics

* **`/recognized_text`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text recognized by the speech recognition, and to be processed by the VLM.

* **`/display_robot_state`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The robot state to be displayed on the robot's LCD.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/sarai-standalone/sarai_speech_recognition/-/issues).
