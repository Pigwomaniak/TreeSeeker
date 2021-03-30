#include <drone_ridder.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "test_DR_square");
    ros::NodeHandle test_DR_square("~");
    std::string rosNamespace = test_DR_square.getNamespace();
    ros::Publisher set_local_pos_pub = test_DR_square.advertise<geometry_msgs::Point>("/drone_ridder/set_local_position", 1);

    ros::Rate rate(.2);
    geometry_msgs::Point waypoint;

    ros::spinOnce();
    rate.sleep();

    waypoint.x = 0;
    waypoint.y = 0;
    waypoint.z = 10;
    set_local_pos_pub.publish(waypoint);
    ROS_INFO("waypoint 1");
    ros::spinOnce();
    rate.sleep();

    waypoint.x = 5;
    set_local_pos_pub.publish(waypoint);
    ROS_INFO("waypoint 2");
    ros::spinOnce();
    rate.sleep();

    waypoint.y = 5;
    set_local_pos_pub.publish(waypoint);
    ROS_INFO("waypoint 3");
    ros::spinOnce();
    rate.sleep();

    waypoint.x = 0;
    set_local_pos_pub.publish(waypoint);
    ROS_INFO("waypoint 4");
    ros::spinOnce();
    rate.sleep();

    waypoint.y = 0;
    set_local_pos_pub.publish(waypoint);
    ROS_INFO("waypoint 5");
    ros::spinOnce();
    rate.sleep();

}

