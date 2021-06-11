//
// Created by maciek on 22.05.2021.
//
#pragma once

#include <ros/ros.h>
#include <trajectory_planer_msgs/TrajectoryPlaner.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

#define FLY_ALT 30
#define DROP_BALL_ALT 5
#define LOW_FLY_ALT 10
#define MAV_STATE_ACTIVE 4
#define MERES_TO_LATITUDE 0.000008983031
#define POSITION_WAYPOINT_ACCURACY 0.5 // in meters
#define DROP_WAYPOINT_ACCURACY 0.3 // in meters

enum class MissionState{
    standby,
    waitingForArm,
    gettingOnMissionStartPlace,
    firstLookAtField,
    goToNextTree,
    dropBall,
    goHome
};

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void trajectory_planer_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg);
void mav_state_cb(const mavros_msgs::State::ConstPtr& msg);
void init_publisher_subscriber(ros::NodeHandle controlNode);
MissionState startMission(sensor_msgs::NavSatFix* takeOffPointWGS84, nav_msgs::Odometry* takeOffPoint);
MissionState getToStartPlace(sensor_msgs::NavSatFix startingPoint, const sensor_msgs::NavSatFix& takeoffPoint);
MissionState getToStartPlace(geometry_msgs::Point startingPoint);
MissionState firstLookAtField(geometry_msgs::Point endPoint);
MissionState goToNextTree();
MissionState dropBall();
MissionState goHome();
geometry_msgs::Point globalToLocalPosition(const sensor_msgs::NavSatFix& global);
double pointDistance(const geometry_msgs::Point& destinationPoint);
double headingToPoint(const geometry_msgs::Point& destinationPoint);
