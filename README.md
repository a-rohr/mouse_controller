# NERMO MOUSE CONTROLLER

This is the ROS package of the control architecture for Nermo. 

## Setup
Before attempting to runs the mouse_controller, ensure that ROS is [installed](http://wiki.ros.org/noetic/Installation), a catkin workspace is set up. For some initial guidance, follow the tutorials offered by the Open Robotics community [here](http://wiki.ros.org/ROS/Tutorials). 

### Setup with upstream input device
The ROS package is currently configured to take inputs from an upstream connected controller (e.g. a PS4 controller) using the [ROS Joy package](http://wiki.ros.org/joy). 

Therefore, before running the mouse_controller, ensure that a controller is connected and the joy node active. Otherwise, the mouse will load, but no movements happen. 

To run the mouse_controller, simulation etc. launch using 
`roslaunch mouse_controller mouse_controller_launch.launch`

### Setup without upstream input device
The mouse_controller also works for basic straight-line tests without a controller. To achieve this, currently some minor modifications need to be made in the `high_level_controller.py` and `motion_module.py`.

Changes in `high_level_controller.py`

Uncomment the line in the main method loop `#vel_in = 0.4`. This parameter can be freely adjusted to change the velocity of the mouse. Essentially, the controller now sends a continous, constant control signal to the motion module.

Changes in `motion_module.py` 

In standard configuration, the mouse is in a non-moving neutral configuration. To ensure the mouse walks without a control input, change the parameter `self.balance_mode` from `True` to `False`. 

## Control Architecture
On a high level, the control architecture of Nermo combines quadruped legged motion control and spine motion control. These two motions are not controlled in isolation and therefore the control architecture ensures synchronization between the leg and spine control. 

## Overview of the ROS Node Architecture
The diagram below provides an overview of the ROS implementation of the control architecture. The diagram shows the different control nodes, message topics, as well as the connection with the physical mouse. 
