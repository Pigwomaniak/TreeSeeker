#include <mission_commander_3colors.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "mission_commander_3colors");
    ros::NodeHandle mission_commander_3colors("~");
    init_publisher_subscriber(mission_commander_3colors);
    ros::Rate rate(2);
    MissionState missionState = MissionState::waitingForArm;
    std::stack<sensor_msgs::NavSatFix> waypoints;
    generateWaypoints(&waypoints, mission_commander_3colors);
    while (ros::ok()){
        ros::spinOnce();
        switch (missionState) {
            case MissionState::waitingForArm:
                missionState = startMission(&takeOffPointWGS84, &takeOffPoint);
                break;
            case MissionState::scanField:
                missionState = scanField(mission_commander_3colors);
                break;
            case MissionState::goToNextObject:
                missionState = goToNextObject(mission_commander_3colors);
                break;
            case MissionState::takeCloseLook:
                missionState = ;
                break;
            case MissionState::goHome:
                missionState = goHome();
                break;
            case MissionState::standby:
                ros::Duration(5).sleep();
                ROS_INFO("Standby");
            default: break;
        }
        rate.sleep();
    }
}