# drone_ridder
ROS package for drone control via mavlink mavros


## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)
* [Features](#features)
* [Status](#status)
* [Inspiration](#inspiration)
* [Contact](#contact)

## General info
Project is about ROS drone motion control and performing autonomous
missions for student competitions and others

## Technologies
* [Robot Operating System](https://www.ros.org/)
* [mavlink](https://mavlink.io/en/)
* [Ardupilot](https://ardupilot.org/)

## Setup
At first install [mavros package](http://wiki.ros.org/mavros#mavros.2FPlugins.Published_Topics-4)
 

To start using clone repository to catkin workspace where you keep source
code and run `catkin build`, next run _mavros_node_ for drone communication.

## Code Examples
To run node for drone motion control:

```
rosrun drone_ridder drone_ridder
```
At another terminal run test sequence for quick demonstration
```
rosrun drone_ridder test_DR_square
```

## Features
Now the main node is [drone_riddeer](./src/drone_ridder.cpp) this node will provide 5 topisc to publish in for drone control.

* "drone_ridder/set_mode" message `std_msgs::String` where y can rquest of mode hange
* "drone_ridder/set_position_offset" message `geometry_msgs::Point` where values x, y, z are vector of movment in ENU.
* "drone_ridder/set_local_position" message `geometry_msgs::Point` where values x, y, z are destination coordinates in local ENU.
* "drone_ridder/heading" message `std_msgs::Float64` where value represent YAW angle. 0 is when x is facing Est and risig value i counter clockwise.
* "drone_ridder/set_global_position" message `geographic_msgs::GeoPoseStamped` set LLA destination possion

At start drone is waiting for user to manually set drone to _guided mode_
next arming procedure is called and drone takes off at altitude _10m_.

The [testing program](./src/test_DR_square.cpp) provide simple square sequence
where drone at first will perform:

* 5m square at current altitude using global position feature
* 5m square at altitude of 10m using local position feature
* 5m square at current altitude using offset position feature

###To-do

* debug global position waypoint setting issue
* idiot IPE
* test in field
## Status
Project is: _in progress_

## Inspiration
Project inspired by [Inteligent-Quads](https://github.com/Intelligent-Quads)

## Contact
Created by [Pigwomaniak](https://github.com/Pigwomaniak) - feel free to contact me!
