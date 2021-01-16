# Cortex-Message-Handling
<img 
src="https://github.com/Taireyune/cortex-message-handling/blob/main/images/communications.png" 
width="782" height="527" alt="relationship">

## Overview
This project (ROS workspace) holds some of the messaging scripts for my robotic arm project. The goal is to allow teleops and software controllers to communicate with the simulation and the real version of the robotic arm. These functionalities are all natively available in ROS. However, since I will be using Unity Game Engine and multiple camera feeds for my applications, I have added these code to achieve said connections.

## Environment
- Ubuntu 18.04
- ROS Melodic

## Unity Engine Plugin
To allow the Linux version of Unity to work with ROS, a thin wrapper of ROS pub/sub, simulator_relay.cpp, was compiled into a library and dropped into the Unity Plugins folder for use. Larger data streams such as the camera views were transferred using ZeroMQ.
Since the simulated robot has a few specific messages to pass, this solution is quick and sufficent. A more general solution may be to use Unity with rosbridge.

<img 
src="https://github.com/Taireyune/cortex-message-handling/blob/main/images/simulator_robotic_arm_circled.png" 
width="744" height="767" alt="simulator relationship">

## Trigger acquired video stream
FLIR Blackfly cameras were used to supply the visual inputs of the real robotic arm. live_binocular_relay.cpp was compiled into an executable to setup the cameras with the appropriate settings and relay the video stream to the agent using ZeroMQ communications.  

Blackfly cameras can capture using hardware or software triggers with global shutters. When multiple blackfly cameras are used, frames are captured by trigger acquisition to ensure minimal staggering between the frames of each camera. Global shutters are used to ensure frames are not distorted during motion. These characterisitcs are put inplace to produce visual inputs that geometrically conforms with those produced by the Unity Engine during high speed motion, reducing the need to heavily regularize variabilities produced by distortion of 3D geometry.

<img 
src="https://github.com/Taireyune/cortex-message-handling/blob/main/images/real_robotic_arm_circled.png" 
width="754" height="753" alt="real arm relationship">

## Why Unity
Unity is one of the most popular game engines available to the public. It supports development and deployment on Linux which allow for easy interaction with the Robotics and Machine learning tools available in Linux. 

The use of a game engine for machine learning simulations leverage the vast and matured resources from the gaming industry such as graphic renderings and game level design. 3D Graphic rendering and dynamically loaded game objects broaden the capability of regularizing frames needed to train vision systems. Game level design allows more complexed and robust reward systems to train agents for better decision making. Unity also has plugin for Mojuco, a popular physics engine for sim2real robots. 

Personally, I wish they make Unity in Python/C++.