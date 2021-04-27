#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <object_global_localizator_msgs/ObjectGlobalPosition.h>




void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void object_detector_cb(const std_msgs::Int8::ConstPtr& msg);
void bounding_boxes_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
