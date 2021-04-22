
#include <object_position_interpreter.h>


sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;
std_msgs::Int8 yolo_found_object;
darknet_ros_msgs::BoundingBoxes yolo_bounding_box;

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
    //ROS_INFO("get global pose");
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
    //ROS_INFO("get global pose");
}
void found_object_cb(const std_msgs::Int8::ConstPtr& msg){
    yolo_found_object = *msg;
}
void bounding_boxes_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    yolo_bounding_box = *msg;
}