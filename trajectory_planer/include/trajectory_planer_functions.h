#pragma once

#include <ros/ros.h>

//#include <sensor_msgs/NavSatFix.h>
//#include <nav_msgs/Odometry.h>
//#include <std_msgs/Int8.h>

#include <object_global_localizator_msgs/ObjectGlobalPosition.h>
#include <object_global_localizator_msgs/ObjectsGlobalPositions.h>
#include <cmath>
#include <TreeObejctPosition.h>



void init_publisher(ros::NodeHandle controlNode);
void new_Point_cb(const object_global_localizator_msgs::ObjectsGlobalPositions::ConstPtr& msg);
void resetReadFlag();
bool checkReadFlag();
void processReadPoints();
