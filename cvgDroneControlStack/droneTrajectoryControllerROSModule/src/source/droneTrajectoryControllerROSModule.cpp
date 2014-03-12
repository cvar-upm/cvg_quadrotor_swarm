#include "droneTrajectoryControllerROSModule.h"

DroneTrajectoryControllerROSModule::DroneTrajectoryControllerROSModule() :
    DroneModule(droneModule::active, 30.0) ,
    drone_trajectory_controller(getId(), stackPath)
    {
    std::cout << "Constructor: DroneTrajectoryControllerROSModule" << std::endl;
    //droneModuleLoggerType = droneModule::non_logger;

    try {
        XMLFileReader my_xml_reader(stackPath+"configs/drone"+ std::to_string(idDrone)+"/trajectory_controller_config.xml");
        set_moduleRate(my_xml_reader.readDoubleValue( {"trajectory_controller_config","module_frequency"} ));
    } catch ( cvg_XMLFileReader_exception &e) {
    throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }

    stay_in_position = false;
}

DroneTrajectoryControllerROSModule::~DroneTrajectoryControllerROSModule() {}

void DroneTrajectoryControllerROSModule::init()  { /* Everything is on the Constructor */ }
void DroneTrajectoryControllerROSModule::close() { /* DroneModule::~DroneModule gets called automatically */ }

bool DroneTrajectoryControllerROSModule::resetValues() {
    stay_in_position = false;
    drone_trajectory_controller.resetValues();
    return true;
}

bool DroneTrajectoryControllerROSModule::startVal() {
    if ( !isStarted() ) {
        resetValues();
        setNavCommandToZero();
    }
    return DroneModule::startVal();
}

bool DroneTrajectoryControllerROSModule::stopVal() {
    setNavCommandToZero();
    return DroneModule::stopVal();
}

bool DroneTrajectoryControllerROSModule::run() {
    DroneModule::run();
    publishControlMode();

    if (!moduleStarted) {
        return false;
    }

    double pitch, roll, dyaw, dz;
    drone_trajectory_controller.getOutput( &pitch, &roll, &dyaw, &dz);
    drone_trajectory_controller.logControllerState( isStarted());
    setNavCommand( pitch, roll, dyaw, dz);
    publishControllerReferences();
    drone_logger_ros_publisher.run();
    return true;
}

void DroneTrajectoryControllerROSModule::open(ros::NodeHandle & nIn, std::string moduleName) {
    //Node
    DroneModule::open(nIn,moduleName);

    //// Topics ///
    // Subscribers
    dronePositionRefSub     = n.subscribe(DRONE_TRAJECTORY_CONTROLLER_POSITION_REF_SUBSCRIPTION, 1, &DroneTrajectoryControllerROSModule::dronePositionRefsSubCallback, this);
    droneSpeedsRefSub       = n.subscribe(DRONE_TRAJECTORY_CONTROLLER_SPEED_REF_SUBSCRIPTION,    1, &DroneTrajectoryControllerROSModule::droneSpeedsRefsSubCallback, this);
    droneTrajectoryAbsRefSub= n.subscribe(DRONE_TRAJECTORY_CONTROLLER_ABS_TRAJ_REF_CMD_SUBSCRIPTION,   1, &DroneTrajectoryControllerROSModule::droneTrajectoryAbsRefCommandCallback, this);
    droneTrajectoryRelRefSub= n.subscribe(DRONE_TRAJECTORY_CONTROLLER_REL_TRAJ_REF_CMD_SUBSCRIPTION,   1, &DroneTrajectoryControllerROSModule::droneTrajectoryRelRefCommandCallback, this);
    droneYawRefCommandSub   = n.subscribe(DRONE_TRAJECTORY_CONTROLLER_YAW_REF_CMD_SUBSCRIPTION, 1, &DroneTrajectoryControllerROSModule::droneYawRefCommandCallback, this);
    droneEstimatedPoseSubs   = n.subscribe(DRONE_TRAJECTORY_CONTROLLER_POSE_SUBSCRIPTION_GMR,      1, &DroneTrajectoryControllerROSModule::droneEstimatedPoseCallback, this);
    droneEstimatedSpeedsSubs = n.subscribe(DRONE_TRAJECTORY_CONTROLLER_SPEEDS_SUBSCRIPTION_GMR,    1, &DroneTrajectoryControllerROSModule::droneEstimatedSpeedsCallback, this);

    // Publishers
    controlModePub      = n.advertise<droneMsgsROS::droneTrajectoryControllerControlMode>(moduleName+"/controlMode", 1);
    drone_command_pitch_roll_publisher = n.advertise<droneMsgsROS::dronePitchRollCmd>(DRONE_TRAJECTORY_CONTROLLER_DRONE_COMMAND_PITCH_ROLL, 1);
    drone_command_dyaw_publisher      = n.advertise<droneMsgsROS::droneDYawCmd>(DRONE_TRAJECTORY_CONTROLLER_DRONE_COMMAND_DYAW, 1);
    drone_command_daltitude_publisher = n.advertise<droneMsgsROS::droneDAltitudeCmd>(DRONE_TRAJECTORY_CONTROLLER_DRONE_COMMAND_DALTITUDE, 1);

    drone_position_reference_publisher   = n.advertise<droneMsgsROS::dronePose>(DRONE_TRAJECTORY_CONTROLLER_POSITION_REF_REBROADCAST_PUBLICATION, 1);
    drone_speed_reference_publisher      = n.advertise<droneMsgsROS::droneSpeeds>(DRONE_TRAJECTORY_CONTROLLER_SPEED_REF_REBROADCAST_PUBLICATION, 1);
    drone_trajectory_reference_publisher = n.advertise<droneMsgsROS::dronePositionTrajectoryRefCommand>(DRONE_TRAJECTORY_CONTROLLER_TRAJECTORY_REF_REBROADCAST_PUBLICATION, 1);

    // Service servers
    setControlModeServerSrv = n.advertiseService(moduleName+"/setControlMode",&DroneTrajectoryControllerROSModule::setControlModeServCall,this);

    drone_logger_ros_publisher.open(n);

    //Flag of module opened
    droneModuleOpened=true;

    //End
    return;
}

void DroneTrajectoryControllerROSModule::dronePositionRefsSubCallback(const droneMsgsROS::dronePositionRefCommandStamped::ConstPtr &msg) {
    drone_trajectory_controller.setPositionRefs_drone_GMR_wrt_GFF( msg->position_command.x, msg->position_command.y, msg->position_command.z);
}

void DroneTrajectoryControllerROSModule::droneSpeedsRefsSubCallback(const droneMsgsROS::droneSpeeds::ConstPtr &msg) {
    drone_trajectory_controller.setHorizontalSpeedRefs_drone_GMR_wrt_GFF( msg->dx, msg->dy);
}

void DroneTrajectoryControllerROSModule::droneTrajectoryAbsRefCommandCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand::ConstPtr& msg) {
    TrajectoryConfiguration traj_config(idDrone);
    DroneTrajectory trajectory(idDrone);

    droneMsgsROS::dronePositionTrajectoryRefCommand last_trajectory_command;
    last_trajectory_command = (*msg);
    int n = last_trajectory_command.droneTrajectory.size();

    if(n==0 && (!stay_in_position) ) {
        droneMsgsROS::dronePositionRefCommand actual_position;
        actual_position.x = +last_estimatedPose.x;
        actual_position.y = +last_estimatedPose.y;
        actual_position.z = +last_estimatedPose.z;
        last_trajectory_command.droneTrajectory.push_back(actual_position);
        last_trajectory_command.is_periodic        = false;
        last_trajectory_command.initial_checkpoint = 0;
        n = 1;
        stay_in_position = true;
    } else {
        if ( n>0 ) {
            stay_in_position = false;
        } else {
            return;
        }
    }

    for (int i=0; i<n; i++ ) {
        double x_abs   = last_trajectory_command.droneTrajectory[i].x;
        double y_abs   = last_trajectory_command.droneTrajectory[i].y;
        double z_abs   = last_trajectory_command.droneTrajectory[i].z;
//        double yaw_abs = last_trajectory_command.droneTrajectory[i].yaw;
        double yaw_abs = 0.0;

        double x2, y2, z2, yaw2, pitch2, roll2;
        drone_trajectory_controller.referenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF( x_abs, y_abs, z_abs, yaw_abs, 0.0, 0.0, x2, y2, z2, yaw2, pitch2, roll2);
        trajectory.addWaypoint( x2, y2, z2);
    }

    trajectory.setPeriodic( last_trajectory_command.is_periodic);
    trajectory.setInitialCheckpoint( last_trajectory_command.initial_checkpoint);
    drone_trajectory_controller.setTrajectory_droneLMrT_wrt_LMrTFF( trajectory, traj_config);
    drone_trajectory_controller.setControlMode(Controller_MidLevel_controlMode::TRAJECTORY_CONTROL);
    return;
}

void DroneTrajectoryControllerROSModule::droneTrajectoryRelRefCommandCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand::ConstPtr &msg)
{
    droneMsgsROS::dronePositionTrajectoryRefCommand last_trajectory_command;
    last_trajectory_command = (*msg);

    TrajectoryConfiguration traj_config(idDrone);
    DroneTrajectory trajectory(idDrone);

    cv::Mat rotation_matrix_yaw;
    rotation_matrix_yaw = cv::Mat::eye(3,3,CV_32F);
    referenceFrames::createRotMatrix_wYvPuR(&rotation_matrix_yaw, last_estimatedPose.yaw, 0.0, 0.0);
    cv::Mat current_absolute_position_vector;
    current_absolute_position_vector.create(3,1,CV_32F);
    current_absolute_position_vector.at<float>(0,0) = last_estimatedPose.x;
    current_absolute_position_vector.at<float>(1,0) = last_estimatedPose.y;
    current_absolute_position_vector.at<float>(2,0) = last_estimatedPose.z;
    cv::Mat relative_position_reference_vector;
    relative_position_reference_vector.create(3,1,CV_32F);
    cv::Mat absolute_position_reference_vector;
    absolute_position_reference_vector.create(3,1,CV_32F);

    int n = last_trajectory_command.droneTrajectory.size();
    if(n==0 && (!stay_in_position) ) {
        droneMsgsROS::dronePositionRefCommand actual_position;
        actual_position.x = 0.0;
        actual_position.y = 0.0;
        actual_position.z = 0.0;
        last_trajectory_command.droneTrajectory.push_back(actual_position);
        last_trajectory_command.is_periodic        = false;
        last_trajectory_command.initial_checkpoint = 0;
        n = 1;
        stay_in_position = true;
    } else {
        if ( n>0 ) {
            stay_in_position = false;
        } else {
            return;
        }
    }

    for ( auto it : last_trajectory_command.droneTrajectory ) {
        relative_position_reference_vector.at<float>(0,0) = it.x;
        relative_position_reference_vector.at<float>(1,0) = it.y;
        relative_position_reference_vector.at<float>(2,0) = it.z;

        absolute_position_reference_vector = current_absolute_position_vector + rotation_matrix_yaw*relative_position_reference_vector;

        double x_abs, y_abs, z_abs;
        x_abs = absolute_position_reference_vector.at<float>(0,0);
        y_abs = absolute_position_reference_vector.at<float>(1,0);
        z_abs = absolute_position_reference_vector.at<float>(2,0);

        double x2, y2, z2, yaw2, pitch2, roll2;
        drone_trajectory_controller.referenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF( x_abs, y_abs, z_abs, 0.0, 0.0, 0.0, x2, y2, z2, yaw2, pitch2, roll2);
        trajectory.addWaypoint( x2, y2, z2);
    }

    trajectory.setPeriodic( last_trajectory_command.is_periodic);
    trajectory.setInitialCheckpoint( last_trajectory_command.initial_checkpoint);
    drone_trajectory_controller.setTrajectory_droneLMrT_wrt_LMrTFF( trajectory, traj_config);
    drone_trajectory_controller.setControlMode(Controller_MidLevel_controlMode::TRAJECTORY_CONTROL);
    return;
}

void DroneTrajectoryControllerROSModule::droneYawRefCommandCallback(const droneMsgsROS::droneYawRefCommand::ConstPtr &msg) {
    drone_trajectory_controller.setYawRef_drone_GMR_wrt_GFF( msg->yaw);
}

void DroneTrajectoryControllerROSModule::droneEstimatedPoseCallback(const droneMsgsROS::dronePose::ConstPtr &msg) {
    last_estimatedPose = (*msg);
    drone_trajectory_controller.setFeedback_drone_pose_GMR_wrt_GFF( msg->x, msg->y, msg->z, msg->yaw, msg->pitch, msg->roll);
}

void DroneTrajectoryControllerROSModule::droneEstimatedSpeedsCallback(const droneMsgsROS::droneSpeeds::ConstPtr &msg) {
    last_estimatedSpeed = (*msg);
    drone_trajectory_controller.setFeedback_drone_speeds_GMR_wrt_GFF( msg->dx, msg->dy, msg->dz, msg->dyaw, msg->dpitch, msg->droll);
    return;
}

void DroneTrajectoryControllerROSModule::publishControllerReferences() {
    // get current_drone_position_reference
    drone_trajectory_controller.getCurrentPositionReference( &current_drone_position_reference.x,
                                                             &current_drone_position_reference.y,
                                                             &current_drone_position_reference.z,
                                                             &current_drone_position_reference.yaw,
                                                             &current_drone_position_reference.pitch,
                                                             &current_drone_position_reference.roll);

    // get current_drone_speed_reference
    drone_trajectory_controller.getCurrentSpeedReference( &current_drone_speed_reference.dx,
                                                          &current_drone_speed_reference.dy,
                                                          &current_drone_speed_reference.dz,
                                                          &current_drone_speed_reference.dyaw);
    current_drone_speed_reference.dpitch = 0.0;
    current_drone_speed_reference.droll  = 0.0;

    // get current_drone_trajectory_command
    std::vector<SimpleTrajectoryWaypoint> trajectory_waypoints;
    int initial_checkpoint_aux;
    bool is_periodic_aux;
    drone_trajectory_controller.getCurrentTrajectoryReference( &trajectory_waypoints,
                                                               &initial_checkpoint_aux,
                                                               &is_periodic_aux);
    current_drone_trajectory_command.initial_checkpoint = initial_checkpoint_aux;
    current_drone_trajectory_command.is_periodic        = is_periodic_aux;
    current_drone_trajectory_command.droneTrajectory.clear();
    int i = 0;
    for (auto it : trajectory_waypoints) {
        droneMsgsROS::dronePositionRefCommand next_waypoint;
        next_waypoint.x = it.x;
        next_waypoint.y = it.y;
        next_waypoint.z = it.z;
        current_drone_trajectory_command.droneTrajectory.push_back(next_waypoint);
        i++;
    }

    if (droneModuleOpened) {
        drone_position_reference_publisher.publish(current_drone_position_reference);
        drone_speed_reference_publisher.publish(current_drone_speed_reference);
        drone_trajectory_reference_publisher.publish(current_drone_trajectory_command);
    }
}

void DroneTrajectoryControllerROSModule::publishControlMode() {
    droneMsgsROS::droneTrajectoryControllerControlMode controlModeMsg;
    controlModeMsg.command = drone_trajectory_controller.getControlMode();

    if (droneModuleOpened)
        controlModePub.publish(controlModeMsg);
}

int DroneTrajectoryControllerROSModule::publishDroneNavCommand() {
    if(droneModuleOpened==false)
        return 0;

    drone_command_pitch_roll_publisher.publish(drone_command_pitch_roll_msg);
    drone_command_dyaw_publisher.publish(drone_command_dyaw_msg);
    drone_command_daltitude_publisher.publish(drone_command_daltitude_msg);
    return 1;
}

void DroneTrajectoryControllerROSModule::setNavCommand(float pitch, float roll, float dyaw, float dz, double time) {
    drone_command_pitch_roll_msg.header.stamp = ros::Time::now();
    drone_command_dyaw_msg.header.stamp       = ros::Time::now();
    drone_command_daltitude_msg.header.stamp  = ros::Time::now();

    drone_command_pitch_roll_msg.pitchCmd    = pitch;
    drone_command_pitch_roll_msg.rollCmd     = roll;
    drone_command_dyaw_msg.dYawCmd           = dyaw;
    drone_command_daltitude_msg.dAltitudeCmd = dz;

    publishDroneNavCommand();
    return;
}

void DroneTrajectoryControllerROSModule::setNavCommandToZero(void) {
    setNavCommand(0.0,0.0,0.0,0.0,-1.0);
    return;
}

bool DroneTrajectoryControllerROSModule::setControlModeServCall(droneMsgsROS::setControlMode::Request& request, droneMsgsROS::setControlMode::Response& response) {
    Controller_MidLevel_controlMode::controlMode new_control_mode;
    switch (request.controlMode.command) {
    case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        new_control_mode = Controller_MidLevel_controlMode::TRAJECTORY_CONTROL;
        break;
    case Controller_MidLevel_controlMode::POSITION_CONTROL:
        new_control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
        break;
    case Controller_MidLevel_controlMode::SPEED_CONTROL:
        new_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
        break;
    default:
        new_control_mode = Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE;
//        throw std::domain_error(std::string("Service call to switch to UNKNOWN_CONTROL_MODE") + std::to_string( (int) new_control_mode));
        break;
    }

    response.ack = drone_trajectory_controller.setControlMode(new_control_mode);
    return response.ack;
}
