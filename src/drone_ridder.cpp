#include <drone_ridder.h>
#include <math.h>
#include <mutex>
#include <queue>
#include <memory>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <stdlib.h>
#include <time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>


void positionOffset_cb(const geometry_msgs::Point::ConstPtr& poseOffset)
{

}


void positionGlobal_cb(const geometry_msgs::Point::ConstPtr& posGlobal)
{

}


void modeChange_cb(const std_msgs::String::ConstPtr& newMode)
{

}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "drone_ridder");
	ros::NodeHandle drone_ridder("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(drone_ridder);
	//ros::Subscriber sub = drone_ridder.subscribe("/tree_seeker/drone_ridder/pos_offset", 1, positionOffset_cb);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(10);



	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		
	}
	land();
	return 0;
}