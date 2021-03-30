#include <drone_ridder.h>
//#include <queue>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

void positionOffset_cb(const geometry_msgs::Point::ConstPtr& msg){
	geometry_msgs::Point pos_offset = *msg;
	activeWaypointLocal = true;
    set_local_destination(current_pose_g.pose.pose.position.x + pos_offset.x,
                          current_pose_g.pose.pose.position.y + pos_offset.y,
                          current_pose_g.pose.pose.position.z + pos_offset.z);
}

void positionGlobalSet_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& msg){
    geographic_msgs::GeoPoseStamped global_pos = *msg;
	activeWaypointLocal = false;
    set_global_destination(global_pos.pose.position.latitude,
                           global_pos.pose.position.longitude,
                           global_pos.pose.position.altitude);
}

void localPositionSet_cb(const geometry_msgs::Point::ConstPtr& msg){
	geometry_msgs::Point localPositionSet = *msg;
    activeWaypointLocal = true;
	set_local_destination(localPositionSet.x, localPositionSet.y, localPositionSet.z);
}

void headingSet_cb(const std_msgs::Float64::ConstPtr& msg){
    std_msgs::Float64 heading = *msg;
    set_heading(heading.data);
}

void modeChange_cb(const std_msgs::String::ConstPtr& msg){
	std_msgs::String mode = *msg;
	set_mode(mode.data);
}

int main(int argc, char** argv){
	//initialize ros 
	ros::init(argc, argv, "drone_ridder");
	ros::NodeHandle drone_ridder("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(drone_ridder);

	ros::Subscriber pos_offset_sub = drone_ridder.subscribe("/drone_ridder/set_position_offset", 1, positionOffset_cb);
	ros::Subscriber pos_global_sub = drone_ridder.subscribe("/drone_ridder/set_global_position", 1, positionGlobalSet_cb);
	ros::Subscriber pos_local_sub = drone_ridder.subscribe("/drone_ridder/set_local_position", 1, localPositionSet_cb);
	ros::Subscriber mode_sub = drone_ridder.subscribe("/drone_ridder/set_mode", 1, modeChange_cb);
    ros::Subscriber heading_sub = drone_ridder.subscribe("/drone_ridder/set_heading", 1, headingSet_cb);
  	// wait for FCU connection
	wait4connect();
	//wait for used to switch to mode GUIDED
	wait4start();
	//request takeoff
	takeoff(10);
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);

	while(ros::ok()){
		ros::spinOnce();
		//check_waypoint_reached();
		rate.sleep();
	}
	return 0;
}