# Draw My Life code

## Overview

This folder contains all the code to run the Draw My Life activity.

<img src="../images/Startscreen.png" width=960 height=546>

## Installation

### Dependencies

Please check the dependency section of each of the ROS2 packages associated with the Draw My Life activity.

### Building

1) Copy all the ROS2 packages associated with the Draw My Life activity into your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the packages with colcon:
    ```
    cd ~/ros2_ws
    colcon build
    ```

## Usage

### Default usage

Open a terminal in your ROS2 workspace and run:
```
source install/setup.bash
ros2 launch pixelbot_vlm draw_my_life_interaction.launch.py
```

### Optional parameters

- **pressure_recording**: Whether to record the pressure or not. Can be used only if a wacom tablet is being used. Enabled by default. Please disable this parameter if you do not have a wacom tablet.
- **vlm_model**: The VLM model to be used. The default value is qwen3-vl:235b-instruct-cloud. This model runs on the cloud infrastructure of ollama. In case you use the ollama api and host the VLM on your own local server, like we did for this study, make sure to export the OLLAMA_HOST variable with the one associated with your server (ex: `export OLLAMA_HOST=http://your_server_ip_address:11434`). In case you rely on the OpenAI api (gpt models), make sure to export your api key (ex: `export OPENAI_API_KEY=your_api_key`).

Here you will find an example of a command making use of the optional parameters:
```
ros2 launch pixelbot_vlm draw_my_life_interaction.launch.py pressure_recording:=disabled vlm_model:=gpt-5-2025-08-07
```


**Note:** Exporting the variables mentioned above can be automated with commands like the ones below:

```
$echo 'export OLLAMA_HOST=http://your_server_ip_address:11434' >> ~/.bashrc
```

or

```
$echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```



## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/SARAI-Lab-KIT/draw_my_life_study_1/issues).
