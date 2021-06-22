# TreeSeeker
Code for autonomous drone control to find sick trees. 

## Table of contents
* [General info](#general-info)
* [Screenshots](#screenshots)
* [Technologies](#technologies)
* [Setup](#setup)
* [Features](#features)
* [Status](#status)
* [Inspiration](#inspiration)
* [Contact](#contact)

## General info
Project is about ROS drone motion control and performing autonomous 
missions for student competitions and others

## Screenshots
[Example simulation view](./img/simulation_drone_image.jpg)

## Technologies
* [Robot Operating System](https://www.ros.org/)
* [Gazebo 11](http://gazebosim.org/)
* [Ardupilot](https://ardupilot.org/)

## Setup
To start using clone repository to catkin workspace where you keep source 
code and ru `catkin build`

This repository is connected with [drone_sim](https://github.com/Pigwomaniak/drone_sim) repository where whole simiulation gazebo files are stored.

## Code Examples
To run whole trees mission in simulation:

```
roslaunch mission_commander trees_sim.launch
```
To run whole 3color mission in real world:
```
roslaunch mission_commander 3color.launch
```

## Features

At start drone is waiting for user to manually set drone to _guided mode_ 
next arming procedure is called and drone takes off at altitude _10m_. 
After that the [mission_commander](./mission_commander) node is managing the mission.
The [config parameters](./mission_commander/config) contains mission parameters for specific mission.

Whole system contains of following nodes:

* [mission_commander](./mission_commander)
* [drone_ridder](./drone_ridder)
* [object_global_localizator](./object_global_localizator)
* [trajectory_planer](./trajectory_planer)
* [ball_droper](https://github.com/Pigwomaniak/ball_droper)

###To-do

* object area measuring
* idiot IPE
* test in field
## Status
Project is: _in progress_

## Inspiration
Project inspired by [Inteligent-Quads](https://github.com/Intelligent-Quads)

## Contact
Created by [Pigwomaniak](https://github.com/Pigwomaniak) and [tom1322s](https://github.com/tom1322s) - feel free to contact!
