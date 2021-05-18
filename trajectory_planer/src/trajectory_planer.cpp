#include <trajectory_planer_functions.h>


unsigned int errorCounter = 0;

int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, "trajectory_planer");
    ros::NodeHandle trajectory_planer("~");

    ros::Subscriber point_sub = trajectory_planer.subscribe("/objec_global_localizator", 1, new_Point_cb);

    //init_publisher(object_global_localizator);

    ros::Rate rate(2.0);
    ROS_INFO("Trajectory planer node started");
    resetReadFlag();

    while(ros::ok())
    {
    	ros::spinOnce();

    	if(checkReadFlag())
    	{
    		processReadPoints();
    	}
    	else
    	{
    		errorCounter++;
    	}


    	resetReadFlag();
    	rate.sleep();
    }
}
