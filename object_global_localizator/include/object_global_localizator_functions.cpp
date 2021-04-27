#include <object_global_localizator.h>

sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;
std_msgs::Int8 objects_in_camera_view;

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
}

void object_detector_cb(const std_msgs::Int8::ConstPtr& msg){
    objects_in_camera_view = *msg;
}

void bounding_boxes_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    darknet_ros_msgs::BoundingBoxes boundingBoxes = *msg;
    for (int i = 0; i < boundingBoxes.bounding_boxes.size(); ++i) {
        boundingBoxes.bounding_boxes[i].probability;
    }

}