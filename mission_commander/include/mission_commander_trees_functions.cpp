//
// Created by maciek on 22.05.2021.
//

#include "mission_commander_trees.h"

sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;
trajectory_planer_msgs::TrajectoryPlaner waypointToTree;
mavros_msgs::State mavState;

ros::Publisher set_local_pos_pub;
ros::Publisher set_heading_pub;
ros::Publisher set_offset_pub;
ros::Publisher set_mode_pub;
ros::Publisher set_global_pos_pub;
ros::Publisher waypoint_reach_pub;

ros::Subscriber global_pose_sub;
ros::Subscriber local_pose_sub;
ros::Subscriber trajectory_planer_sub;
ros::Subscriber mav_state_sub;


void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
}

void trajectory_planer_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg){
    waypointToTree = *msg;
}
void mav_state_cb(const mavros_msgs::State::ConstPtr& msg){
    mavState = *msg;
}


void init_publisher_subscriber(ros::NodeHandle controlNode){
    // Publishers
    set_local_pos_pub = controlNode.advertise<geometry_msgs::Point>("/drone_ridder/set_local_position", 1);
    set_heading_pub = controlNode.advertise<std_msgs::Float64>("/drone_ridder/set_heading", 1);
    set_offset_pub = controlNode.advertise<geometry_msgs::Point>("/drone_ridder/set_position_offset", 1);
    set_mode_pub = controlNode.advertise<std_msgs::String>("/drone_ridder/set_mode", 1);
    set_global_pos_pub = controlNode.advertise<geographic_msgs::GeoPoseStamped>("/drone_ridder/set_global_position", 1);
    waypoint_reach_pub = controlNode.advertise<trajectory_planer_msgs::TrajectoryPlaner>("/trajectory_planer/waypoint_reach", 1);
    //Subscribers
    global_pose_sub = controlNode.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    local_pose_sub = controlNode.subscribe("/mavros/global_position/local", 1, local_pos_cb);
    trajectory_planer_sub = controlNode.subscribe("/trajectory_planer/next_waypoint", 1, trajectory_planer_cb);
    mav_state_sub = controlNode.subscribe("/mavros/state", 1, mav_state_cb);
}

MissionState startMission(sensor_msgs::NavSatFix* takeOffPointWGS84, nav_msgs::Odometry* takeOffPoint){
    // Wait until drone starts
    if (mavState.system_status != MAV_STATE_ACTIVE){
        ros::Duration(0.5).sleep();
        return MissionState::waitingForArm;
    }
    *takeOffPointWGS84 = global_position;
    *takeOffPoint = local_position;
    ROS_INFO("ARMED");
    return MissionState::gettingOnMissionStartPlace;
}

// do dokończenia
MissionState getToStartPlace(sensor_msgs::NavSatFix startingPoint, const sensor_msgs::NavSatFix& takeoffPoint){
    geographic_msgs::GeoPoseStamped waypoint;
    std_msgs::Float64 heading;
    heading.data = headingToPoint(globalToLocalPosition(startingPoint));
    waypoint.pose.position.altitude = takeoffPoint.altitude + FLY_ALT;
    waypoint.pose.position.longitude = startingPoint.longitude;
    waypoint.pose.position.latitude = startingPoint.latitude;
    set_heading_pub.publish(heading);
    set_global_pos_pub.publish(waypoint);
    return MissionState::gettingOnMissionStartPlace;
}

MissionState getToStartPlace(geometry_msgs::Point startingPoint){
   if(local_position.pose.pose.position.z < 5){
       return MissionState::gettingOnMissionStartPlace;
   }
    std_msgs::Float64 heading;
    heading.data = headingToPoint(startingPoint);
    set_heading_pub.publish(heading);
    set_local_pos_pub.publish(startingPoint);
    if(pointDistance(startingPoint) < POSITION_WAYPOINT_ACCURACY){
        ROS_INFO("Start point reach, going to next tree");
        return MissionState::goToNextTree;
    } else {
        return MissionState::gettingOnMissionStartPlace;
    }
}

MissionState goToNextTree(){
    geometry_msgs::Point waypoint;
    waypoint.x = waypointToTree.pos1;
    waypoint.y = waypointToTree.pos2;
    waypoint.z = FLY_ALT;
    if(pointDistance(waypoint) > POSITION_WAYPOINT_ACCURACY){
        set_local_pos_pub.publish(waypoint);
        //ROS_INFO("going to %f, %f, %f", waypoint.x, waypoint.y, waypoint.z);
    } else{
        ROS_INFO("Tree reach: x: %f, y: %f", waypoint.x, waypoint.y);
        ros::Duration(1).sleep();
        return MissionState::dropBall;
        //waypoint_reach_pub.publish(waypointToTree);
    }
    return MissionState::goToNextTree;
}

MissionState dropBall(){
    geometry_msgs::Point waypoint;
    waypoint.x = waypointToTree.pos1;
    waypoint.y = waypointToTree.pos2;
    waypoint.z = DROP_BALL_ALT;
    set_local_pos_pub.publish(waypoint);
    if(pointDistance(waypoint) > DROP_WAYPOINT_ACCURACY){
        return MissionState::dropBall;
    } else{
        ros::Duration(2).sleep();
        //ball drop execute
        ros::spinOnce();
        ros::Duration(2).sleep();
        waypoint_reach_pub.publish(waypointToTree);
        ros::spinOnce();
    }
    return MissionState::goToNextTree;
}

MissionState searchForTrees(){
    return MissionState::goToNextTree;
}

geometry_msgs::Point globalToLocalPosition(const sensor_msgs::NavSatFix& global){
    geometry_msgs::Point local;
    local.z = global.altitude - (global_position.altitude - local_position.pose.pose.position.z);
    local.x = local_position.pose.pose.position.x + (global.longitude - global_position.longitude) / (360.0 / 40075000 / cos(global_position.latitude * M_PI / 360));
    local.y = local_position.pose.pose.position.y + (global.latitude - global_position.latitude) / MERES_TO_LATITUDE;
    return local;
}

double pointDistance(const geometry_msgs::Point& destinationPoint){
    double dx = destinationPoint.x - local_position.pose.pose.position.x;
    double dy = destinationPoint.y - local_position.pose.pose.position.y;
    double dz = destinationPoint.z - local_position.pose.pose.position.z;
    return (sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2)));
}

double headingToPoint(const geometry_msgs::Point& destinationPoint){
    double dx = destinationPoint.x - local_position.pose.pose.position.x;
    double dy = destinationPoint.y - local_position.pose.pose.position.y;
    double angle = atan2(dy, dx);
    double heading;
    if (angle >= 0){
        heading = angle / M_PI * 180;
    } else{
        heading = (-angle / M_PI * 180) + 180;
    }
    return heading;
}
