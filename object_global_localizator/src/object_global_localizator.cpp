#include <object_global_localizator.h>



int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, "object_global_localizator");
    ros::NodeHandle object_global_localizator("~");

    ros::Subscriber global_pose_sub = object_global_localizator.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    ros::Subscriber local_pose_sub = object_global_localizator.subscribe("/mavros/global_position/local", 1, local_pos_cb);
    ros::Subscriber YOLO_object_detector_sub = object_global_localizator.subscribe("darknet_ros/object_detector", 1, object_detector_cb);
    ros::Subscriber YOLO_bounding_boxes_sub = object_global_localizator.subscribe("darknet_ros/bounding_boxes", 1, bounding_boxes_cb);
    ros::Publisher object_global_position_pub = object_global_localizator.advertise<object_global_localizator_msgs::ObjectGlobalPosition>("/objec_global_localizator", 1);

    ros::spin();
}
