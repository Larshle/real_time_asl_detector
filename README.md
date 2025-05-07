# Real-Time ASL Detector and Remote Command Executor

## Overview

This project captures American Sign Language (ASL) gestures in real time via webcam, assembles detected letters into words and sentences, and upon a final "execute" trigger (letter `E`), uses a Large Language Model (LLM) to translate the assembled sentence and generates into a new ROS command script. The generated code is then sent over SSH to a remote Linux machine, where it executes within a ROS 1 workspace to control a TurtleBot.

Key components:

* **ASL Detection**: Uses a YOLO-based inference model to detect ASL letters from camera frames.
* **Sentence Assembler**: Buffers letters into words and triggers an execution step when the letter `E` is detected.
* **LLM-based Code Generator**: Feeds the assembled sentence into a large language model, which generates a new Python/ROS command script tailored for the TurtleBot.
* **Command Generator**: Converts LLM output into a Python script via `generate_and_save_command`.
* **Remote Deployment**: Uses `scp` and `ssh` to copy and execute the generated script on a remote ROS-enabled Linux PC.

## Repository Structure

```
Real_time_asl_detector/
├── bcast/
│   └── cast.py            # Handles copying & remote execution
├── scripts/
│   └── remote_asl_cam.py  # Main camera & inference loop
├── asl_cam/
│   ├── camera.py          # Webcam interface
│   ├── letter_builder.py  # Sentence assembly logic
│   └── handler.py         # Command script generator
├── asl_pkg/               # ROS package (on remote host)
│   └── scripts/           # Remote scripts directory
│       └── command.py
├── .env                   # API keys for inference
└── README.md              # This file
```
