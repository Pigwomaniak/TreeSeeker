

int main(int argc, char** argv){

    ros::init(argc, argv, "mission_commander_3colors");
    ros::NodeHandle mission_commander_3colors("~");
    init_publisher_subscriber(mission_commander_3colors);
    ros::Rate rate(2);
    MissionState missionState = MissionState::waitingForArm;
    while (ros::ok()){
        ros::spinOnce();
        switch (missionState) {
            case MissionState::waitingForArm:
                missionState = startMission(&takeOffPointWGS84, &takeOffPoint);
                break;
            case MissionState::gettingOnMissionStartPlace:
                missionState = getToStartPlace(mission_commander_trees);
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