#include <object_position_interpreter.h>





int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, "object_position_interpreter");
    ros::NodeHandle object_position_interpreter("~");

    ros::Subscriber global_pose_sub = object_position_interpreter.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    ros::Subscriber local_pose_sub = object_position_interpreter.subscribe("/mavros/global_position/local", 1, local_pos_cb);
    ros::Subscriber yolo_object_detector_sub = object_position_interpreter.subscribe("/darknet_ros/found_object", 1, found_object_cb);
    ros::Subscriber yolo_bounding_boxes_sub = object_position_interpreter.subscribe("/darknet_ros/bounding_boxes", 1, bounding_boxes_cb);

    ros::Publisher object_position_pub;

}

