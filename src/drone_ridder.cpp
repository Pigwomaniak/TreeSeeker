#include <drone_ridder.h>
//#include <queue>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

void positionOffset_cb(const geometry_msgs::Point::ConstPtr& msg){
	geometry_msgs::Point pos_offset = *msg;
}

void positionGlobal_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
	sensor_msgs::NavSatFix global_pos = *msg;
}

void localPositionSet_cb(const geometry_msgs::Point::ConstPtr& msg){
	geometry_msgs::Point localPositionSet = *msg;
	set_local_destination(localPositionSet.x, localPositionSet.y, localPositionSet.z);
	ROS_INFO("get LPS");
}

void headingSet_cb(const std_msgs::Float64::ConstPtr& msg){
    std_msgs::Float64 heading = *msg;
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
	ros::Subscriber pos_global_sub = drone_ridder.subscribe("/drone_ridder/set_global_position", 1, positionGlobal_cb);
	ros::Subscriber pos_local_sub = drone_ridder.subscribe("/drone_ridder/set_local_position", 1, localPositionSet_cb);
	ros::Subscriber mode_sub = drone_ridder.subscribe("/drone_ridder/set_mode", 1, modeChange_cb);
    ros::Subscriber heading_sub = drone_ridder.subscribe("/drone_ridder/heading", 1, headingSet_cb);
  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame

	//request takeoff
	takeoff(10);
/*
	std::vector<simple_waypoint> waypointList;
	simple_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 5;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 15;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 270;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 5;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


*/
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);

	
	int counter = 0;
	while(ros::ok()){
		ros::spinOnce();
		check_waypoint_reached();
		rate.sleep();
	}
	return 0;
}