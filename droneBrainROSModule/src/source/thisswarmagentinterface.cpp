#include "thisswarmagentinterface.h"

ThisSwarmAgentInterface::ThisSwarmAgentInterface() :
    state_estimator( std::string(MODULE_NAME_ODOMETRY_STATE_ESTIMATOR), ModuleNames::ODOMETRY_STATE_ESTIMATOR),
    trajectory_controller( std::string(MODULE_NAME_TRAJECTORY_CONTROLLER), ModuleNames::TRAJECTORY_CONTROLLER),
    arucoeye( std::string(MODULE_NAME_ARUCO_EYE), ModuleNames::ARUCO_EYE),
    localizer( std::string(MODULE_NAME_LOCALIZER), ModuleNames::LOCALIZER),
    obstacle_processor( std::string(MODULE_NAME_OBSTACLE_PROCESSOR), ModuleNames::OBSTACLE_PROCESSOR),
    trajectory_planner( std::string(MODULE_NAME_TRAJECTORY_PLANNER), ModuleNames::TRAJECTORY_PLANNER),
    yaw_planner( std::string(MODULE_NAME_YAW_PLANNER), ModuleNames::YAW_PLANNER),
    mission_planner( std::string(MODULE_NAME_MISSION_PLANNER), ModuleNames::MISSION_PLANNER)
{
    last_navdata_timestamp = ros::Time( 0, 0);
    wifi_is_ok = false;
    battery_threshold = SWARM_AGENT_BATTERY_LEVEL_CHECK_THRESHOLD;
    return;
}

ThisSwarmAgentInterface::~ThisSwarmAgentInterface() {
    return;
}

void ThisSwarmAgentInterface::open(ros::NodeHandle & nIn) {
    n = nIn;

    state_estimator.open(n);
    trajectory_controller.open(n);
    arucoeye.open(n);
    localizer.open(n);
    obstacle_processor.open(n);
    trajectory_planner.open(n);
    yaw_planner.open(n);
    mission_planner.open(n);

//    navdataSub = n.subscribe("ardrone/navdata", 1, &ThisSwarmAgentInterface::navdataSubCallback, this);
    drone_rotation_angles_subscriber = n.subscribe(DRONE_BRAIN_SENSOR_ROTATION_ANGLES, 1, &ThisSwarmAgentInterface::droneRotationAnglesCallback, this);
    drone_altitude_subscriber = n.subscribe(DRONE_BRAIN_SENSOR_ALTITUDE, 1, &ThisSwarmAgentInterface::droneAltitudeCallback, this);
    drone_ground_optical_flow_subscriber = n.subscribe(DRONE_BRAIN_SENSOR_GROUND_SPEED, 1, &ThisSwarmAgentInterface::droneGroundOpticalFlowCallback, this);
    battery_level_subscriber=n.subscribe(DRONE_BRAIN_SENSOR_BATTERY, 1, &ThisSwarmAgentInterface::batteryCallback, this);
    drone_status_subscriber=n.subscribe(DRONE_BRAIN_SENSOR_STATUS, 1, &ThisSwarmAgentInterface::droneStatusCallback, this);

//    takeOffPubl = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
//    landPubl = n.advertise<std_msgs::Empty>("ardrone/land", 1);
//    resetPubl = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
//    commandsPubl = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    DroneCommandPubl=n.advertise<droneMsgsROS::droneCommand>(DRONE_DRIVER_COMMAND_DRONE_HL_COMMAND,1, true);
    estimatedPoseSub = n.subscribe(DRONE_BRAIN_POSE_SUBSCRIBER, 1, &ThisSwarmAgentInterface::estimatedPoseSubCallback, this);
}

bool ThisSwarmAgentInterface::startModule( ModuleNames::name module_name_enum) {
    switch (module_name_enum) {
    case ModuleNames::ODOMETRY_STATE_ESTIMATOR:
        return state_estimator.start();
        break;
    case ModuleNames::TRAJECTORY_CONTROLLER:
        return trajectory_controller.start();
        break;
    case ModuleNames::ARUCO_EYE:
        return arucoeye.start();
        break;
    case ModuleNames::LOCALIZER:
        return localizer.start();
        break;
    case ModuleNames::OBSTACLE_PROCESSOR:
        return obstacle_processor.start();
        break;
    case ModuleNames::TRAJECTORY_PLANNER:
        return trajectory_planner.start();
        break;
    case ModuleNames::YAW_PLANNER:
        return yaw_planner.start();
        break;
    case ModuleNames::MISSION_PLANNER:
        return mission_planner.start();
        break;
    default:
        return false;
        break;
    }
}

bool ThisSwarmAgentInterface::stopModule( ModuleNames::name module_name_enum) {
    switch (module_name_enum) {
    case ModuleNames::ODOMETRY_STATE_ESTIMATOR:
        return state_estimator.stop();
        break;
    case ModuleNames::TRAJECTORY_CONTROLLER:
        return trajectory_controller.stop();
        break;
    case ModuleNames::ARUCO_EYE:
        return arucoeye.stop();
        break;
    case ModuleNames::LOCALIZER:
        return localizer.stop();
        break;
    case ModuleNames::OBSTACLE_PROCESSOR:
        return obstacle_processor.stop();
        break;
    case ModuleNames::TRAJECTORY_PLANNER:
        return trajectory_planner.stop();
        break;
    case ModuleNames::YAW_PLANNER:
        return yaw_planner.stop();
        break;
    case ModuleNames::MISSION_PLANNER:
        return mission_planner.stop();
        break;
    default:
        return false;
        break;
    }
}

bool ThisSwarmAgentInterface::resetModule( ModuleNames::name module_name_enum) {
    switch (module_name_enum) {
    case ModuleNames::ODOMETRY_STATE_ESTIMATOR:
        return state_estimator.reset();
        break;
    case ModuleNames::TRAJECTORY_CONTROLLER:
        return trajectory_controller.reset();
        break;
    case ModuleNames::ARUCO_EYE:
        return arucoeye.reset();
        break;
    case ModuleNames::LOCALIZER:
        return localizer.reset();
        break;
    case ModuleNames::OBSTACLE_PROCESSOR:
        return obstacle_processor.reset();
        break;
    case ModuleNames::TRAJECTORY_PLANNER:
        return trajectory_planner.reset();
        break;
    case ModuleNames::YAW_PLANNER:
        return yaw_planner.reset();
        break;
    case ModuleNames::MISSION_PLANNER:
        return mission_planner.reset();
        break;
    default:
        return false;
        break;
    }
}


bool ThisSwarmAgentInterface::isStartedModule( ModuleNames::name module_name_enum) {
    switch (module_name_enum) {
    case ModuleNames::ODOMETRY_STATE_ESTIMATOR:
        return state_estimator.isStarted();
        break;
    case ModuleNames::TRAJECTORY_CONTROLLER:
        return trajectory_controller.isStarted();
        break;
    case ModuleNames::ARUCO_EYE:
        return arucoeye.isStarted();
        break;
    case ModuleNames::LOCALIZER:
        return localizer.isStarted();
        break;
    case ModuleNames::OBSTACLE_PROCESSOR:
        return obstacle_processor.isStarted();
        break;
    case ModuleNames::TRAJECTORY_PLANNER:
        return trajectory_planner.isStarted();
        break;
    case ModuleNames::YAW_PLANNER:
        return yaw_planner.isStarted();
        break;
    case ModuleNames::MISSION_PLANNER:
        return mission_planner.isStarted();
        break;
    default:
        return false;
        break;
    }
}

//void ThisSwarmAgentInterface::navdataSubCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
//    last_navdata_timestamp = ros::Time::now();
//    wifi_is_ok = true;
//    last_navdata = (*msg);
//    last_droneNavData.altitude =-last_navdata.altd/1000.0;
//    last_droneNavData.pitch    =-last_navdata.rotY;
//    last_droneNavData.roll     = last_navdata.rotX;
//    last_droneNavData.yaw      =-last_navdata.rotZ;
//    last_droneNavData.speedX   = last_navdata.vx/1000.0;
//    last_droneNavData.speedY   =-last_navdata.vy/1000.0;
//    last_droneNavData.time     = last_navdata.tm;
//    // See comment on ardrone_autonomy/msg/Navdata.msg
//    // # 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
//    // # 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
//    // # Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
////    last_navdata.state =
//    return;
//}

void ThisSwarmAgentInterface::droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg) {
    last_navdata_timestamp = ros::Time::now();
    wifi_is_ok = true;

    last_rotation_angles_msg = (msg);
}

void ThisSwarmAgentInterface::droneAltitudeCallback(const droneMsgsROS::droneAltitude& msg) {
    last_altitude_msg = (msg);
}

void ThisSwarmAgentInterface::droneGroundOpticalFlowCallback(const droneMsgsROS::vector2Stamped& msg) {
    last_ground_optical_flow_msg = (msg);
}

void ThisSwarmAgentInterface::batteryCallback(const droneMsgsROS::battery::ConstPtr& msg) {
    last_battery_msg = (*msg);
}

void ThisSwarmAgentInterface::droneStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg) {
    // See comment on ardrone_autonomy/msg/Navdata.msg
    // # 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
    // # 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
    // # Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
    last_drone_status_msg = (*msg);
}

bool ThisSwarmAgentInterface::isWifiOk() {
    ros::Time current_time = ros::Time::now();
    if ( (current_time - last_navdata_timestamp).toSec() > SWARM_AGENT_WIFI_TIMEOUT_THRESHOLD ) {
        wifi_is_ok = false;
    }

    return wifi_is_ok;
}

void ThisSwarmAgentInterface::drone_takeOff() {
    droneMsgsROS::droneCommand DroneCommandMsgs;
    DroneCommandMsgs.command = DroneCommandMsgs.TAKE_OFF;
    DroneCommandPubl.publish(DroneCommandMsgs);
}

void ThisSwarmAgentInterface::drone_land() {
    droneMsgsROS::droneCommand DroneCommandMsgs;
    DroneCommandMsgs.command = DroneCommandMsgs.LAND;
    DroneCommandPubl.publish(DroneCommandMsgs);
}

void ThisSwarmAgentInterface::drone_reset() {
    droneMsgsROS::droneCommand DroneCommandMsgs;
    DroneCommandMsgs.command = DroneCommandMsgs.RESET;
    DroneCommandPubl.publish(DroneCommandMsgs);
}

void ThisSwarmAgentInterface::drone_hover() {
    droneMsgsROS::droneCommand DroneCommandMsgs;
    DroneCommandMsgs.command = DroneCommandMsgs.HOVER;
    DroneCommandPubl.publish(DroneCommandMsgs);
}

void ThisSwarmAgentInterface::drone_move() {
    droneMsgsROS::droneCommand DroneCommandMsgs;
    DroneCommandMsgs.command = DroneCommandMsgs.MOVE;
    DroneCommandPubl.publish(DroneCommandMsgs);
}

void ThisSwarmAgentInterface::estimatedPoseSubCallback(const droneMsgsROS::dronePose::ConstPtr& msg) {
    last_estimatedPose = (*msg);
    return;
}

bool ThisSwarmAgentInterface::batteryCheckIsOk() {
    return last_battery_msg.batteryPercent > SWARM_AGENT_BATTERY_LEVEL_CHECK_THRESHOLD;
}
