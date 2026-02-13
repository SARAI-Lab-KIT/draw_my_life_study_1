# pixelbot_vlm

## Overview

This package allows to process the children drawings and verbal input in the context of the draw my life activity.

**Keywords:**  human-robot interaction, Vision Language Model (VLM)

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

- [Robot Operating System (ROS2)](https://docs.ros.org/en/jazzy/index.html) (middleware for robotics).
- [OpenCV](https://pypi.org/project/opencv-python/): Computer vision Python library.
    ```
	sudo pip3 install opencv-python
    ```    
- [Ollama](https://pypi.org/project/ollama/), [OpenAI](https://pypi.org/project/openai/): VLM Python libraries.
    ```
	sudo pip3 install ollama
    sudo pip3 install openai
    ```    


#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_vlm
    ```

## Usage

**IMPORTANT:** Before usage, depending on your way to access the VLM (openai/ollama), you might need to specify your API key as an environment variable. For example:

```
$echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```

You can run the main node with:
```
ros2 run pixelbot_vlm pixelbot_vlm_node
```

## Nodes

### sarai_vlm_node

Allows to process the children drawings and verbal input in the context of the draw my life activity.

#### Subscribed Topics

* **`/recognized_text`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text recognized by the speech recognition, and to be processed by the VLM.

* **`/drawings`** ([sensor_msgs/Image](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html))

	The latest version of the user drawing, to be processed by the VLM.

* **`/child_session_folder_path`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The path of the user session, for data saving purposes.

* **`/reset_drawing_session`** ([std_msgs/Empty](https://docs.ros2.org/foxy/api/std_msgs/msg/Empty.html))

	The reset signal when a Draw My Life activity is finished.


#### Published Topics

* **`/tts_input`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text to be said by the TTS engine.

* **`/display_robot_state`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The robot state to be displayed on the robot's LCD.

#### Parameters

* **`vlm_model`** (string)

	The name of the VLM model to be used. In our case, we use Ollama to interact with an open-source VLM stored on our local server.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/SARAI-Lab-KIT/draw_my_life_study_1/issues).
