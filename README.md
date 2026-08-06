# autonomous-object-retrieval-robot
Autonomous vision-guided robot that detects, navigates to, picks up, and transports objects to the closest human.

# Autonomous Object Retrieval Robot

An autonomous vision-guided robotic system built on the Yahboom Transbot platform. The robot uses computer vision to detect objects, track their position, navigate toward targets, and perform autonomous object retrieval using a robotic arm.

## Project Overview

This project integrates computer vision, mobile robotics, and robotic manipulation to create an autonomous pick-and-place system. The robot uses a camera feed processed with OpenCV and object detection algorithms to identify objects in its environment. It then calculates the object's position relative to the robot, adjusts its movement to approach the target, and uses its robotic arm to pick up and transport objects to a user-selected destination.

## Features

- Real-time computer vision processing using OpenCV
- Object detection and tracking through camera input
- Autonomous robot movement based on visual feedback
- Robotic arm integration for object pickup
- User interaction through Jupyter widgets
- Designed for the Yahboom Transbot robotic platform

## System Workflow

1. Camera captures live video input
2. Computer vision identifies the target object
3. The robot calculates object position and orientation
4. Mobile chassis moves toward the object
5. Robotic arm performs pickup operation
6. Object is transported to the selected destination

## Technologies Used

- Python
- OpenCV
- NumPy
- Computer Vision
- Yahboom Transbot SDK (`Transbot_Lib`)
- Jupyter Notebook
- Raspberry Pi
- Docker

## Hardware Requirements

- Yahboom Transbot robotic platform
- Raspberry Pi
- Camera module
- Robotic arm
- Yahboom motor controller/chassis

## Software Requirements

- Docker environment
- Jupyter Notebook
- ROS
- Python 3
- Yahboom Transbot SDK (`Transbot_Lib`)
