# TreeSeeker
Code for autonomus drone to find sick trees 

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
Add more general information about project. What the purpose of the project is? Motivation?

## Screenshots
[Example screenshot](./img/screenshot.png)

## Technologies
* Ros noetic
* Gazebo 11
* Ardupilot

## Setup
To start using clone repository to catkin workspace where you keep source code and ru `catkin build`

This repository is connected with [drone_sim](https://github.com/Pigwomaniak/drone_sim) repository where whole simiulation gazebo files are stored.

## Code Examples
To run example program:
`$ rosrun tree_seeker square`

## Features
This is erly stage of development and none of features are well tested but few functions seems to work fine
* creating local coordinate system
* adding waypoints in local coordinate system
* addin waypoints in adrucopter local coordinate system
* arming
* flight mode hange

To-do list:
* optimalisation 
* Wow improvement to be done 2

## Status
Project is: _in progress_, _finished_, _no longer continue_ and why?

## Inspiration
Add here credits. Project inspired by..., based on...

## Contact
Created by [@flynerdpl](https://www.flynerd.pl/) - feel free to contact me!
