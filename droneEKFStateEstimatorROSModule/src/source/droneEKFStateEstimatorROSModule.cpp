#include "droneEKFStateEstimatorROSModule.h"
#include "xmlfilereader.h"

DroneEKFStateEstimatorROSModule::DroneEKFStateEstimatorROSModule() :
    DroneModule(droneModule::active, 30.0) ,
    drone_EKF_state_estimator(getId(),stackPath)
    {
    try {
        //droneModuleLoggerType = droneModule::non_logger;

        XMLFileReader my_xml_reader(stackPath +"configs/drone"+ std::to_string(idDrone)+"/ekf_state_estimator_config.xml");
        set_moduleRate(my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","module_frequency"} ));

        ekf_mahalanobis_distance = -1.0;
        ekf_mahalanobis_distance = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_estimation_mahalanobis_distance"} );
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

DroneEKFStateEstimatorROSModule::~DroneEKFStateEstimatorROSModule() {
    close();
}

int DroneEKFStateEstimatorROSModule::publishEstimatedPose(void) {
    if(droneModuleOpened==false)
        return 0;

    drone_estimated_LMrT_pose_publisher.publish(last_drone_estimated_LMrTwrtEKF_pose_msg);
    drone_estimated_GMR_pose_publisher.publish(last_drone_estimated_GMRwrtGFF_pose_msg);
    return 1;
}

int DroneEKFStateEstimatorROSModule::publishEstimatedSpeeds(void) {
    if(droneModuleOpened==false)
        return 0;

    drone_estimated_LMrT_speeds_publisher.publish(last_drone_estimated_LMrTwrtEKF_speeds_msg);
    drone_estimated_GMR_speeds_publisher.publish(last_drone_estimated_GMRwrtGFF_speeds_msg);
    return 1;
}

void DroneEKFStateEstimatorROSModule::open(ros::NodeHandle &nIn, std::string moduleName) {
    //Node
    DroneModule::open(nIn,moduleName);

    //// Topics ///
    //Subscribers
    drone_rotation_angles_subscriber = n.subscribe(DRONE_STATE_ESTIMATOR_SENSOR_ROTATION_ANGLES, 1, &DroneEKFStateEstimatorROSModule::droneRotationAnglesCallback, this);
    drone_altitude_subscriber = n.subscribe(DRONE_STATE_ESTIMATOR_SENSOR_ALTITUDE, 1, &DroneEKFStateEstimatorROSModule::droneAltitudeCallback, this);
    drone_ground_optical_flow_subscriber = n.subscribe(DRONE_STATE_ESTIMATOR_SENSOR_GROUND_SPEED, 1, &DroneEKFStateEstimatorROSModule::droneGroundOpticalFlowCallback, this);
    drone_command_pitch_roll_subscriber = n.subscribe(DRONE_STATE_ESTIMATOR_COMMAND_DRONE_COMMAND_PITCH_ROLL, 1, &DroneEKFStateEstimatorROSModule::droneDroneCommandPitchRollCallback, this);
    drone_commdan_dyaw_subscriber       = n.subscribe(DRONE_STATE_ESTIMATOR_COMMAND_DRONE_COMMAND_DYAW, 1, &DroneEKFStateEstimatorROSModule::droneDroneCommandDYawCallback, this);
    drone_command_daltitude_subscriber  = n.subscribe(DRONE_STATE_ESTIMATOR_COMMAND_DRONE_COMMAND_DALTITUDE, 1, &DroneEKFStateEstimatorROSModule::droneDroneCommandDAltitudeCallback, this);


    // Publishers
    drone_estimated_LMrT_pose_publisher   = n.advertise<droneMsgsROS::dronePose>(DRONE_STATE_ESTIMATOR_POSE_PUBLICATION_LMrT, 1);
    drone_estimated_GMR_pose_publisher    = n.advertise<droneMsgsROS::dronePose>(DRONE_STATE_ESTIMATOR_POSE_PUBLICATION_GMR, 1);
    drone_estimated_LMrT_speeds_publisher = n.advertise<droneMsgsROS::droneSpeeds>(DRONE_STATE_ESTIMATOR_SPEEDS_PUBLICATION_LMrT, 1);
    drone_estimated_GMR_speeds_publisher  = n.advertise<droneMsgsROS::droneSpeeds>(DRONE_STATE_ESTIMATOR_SPEEDS_PUBLICATION_GMR, 1);

    // Service server
    setDroneYawInitSrv     = nIn.advertiseService(std::string(MODULE_NAME_ODOMETRY_STATE_ESTIMATOR)+"/setInitDroneYaw", &DroneEKFStateEstimatorROSModule::setInitDroneYaw,this);

    drone_logger_ros_publisher.open(n);

    //Flag of module opened
    droneModuleOpened=true;

    //End
    return;
}

void DroneEKFStateEstimatorROSModule::droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg) {
    last_rotation_angles_msg = msg;
    drone_EKF_state_estimator.setDroneMeasurementRotationAngles(msg.vector.x, msg.vector.y, msg.vector.z);
    return;
}

void DroneEKFStateEstimatorROSModule::droneAltitudeCallback(const droneMsgsROS::droneAltitude& msg) {
    last_altitude_msg = msg;
    drone_EKF_state_estimator.setDroneMeasurementAltitude(msg.altitude);
    return;
}

void DroneEKFStateEstimatorROSModule::droneGroundOpticalFlowCallback(const droneMsgsROS::vector2Stamped& msg) {
    last_ground_optical_flow_msg = msg;
    drone_EKF_state_estimator.setDroneMeasurementGroundOpticalFlow(msg.vector.x, msg.vector.y);
    return;
}

void DroneEKFStateEstimatorROSModule::droneDroneCommandPitchRollCallback(const droneMsgsROS::dronePitchRollCmd& msg) {
    last_drone_command_pitch_roll_msg = msg;
    drone_EKF_state_estimator.setDroneCommands( last_drone_command_pitch_roll_msg.pitchCmd,
                                                last_drone_command_pitch_roll_msg.rollCmd,
                                                last_drone_command_dyaw_msg.dYawCmd,
                                                last_drone_command_daltitude_msg.dAltitudeCmd);
    return;
}

void DroneEKFStateEstimatorROSModule::droneDroneCommandDYawCallback(const droneMsgsROS::droneDYawCmd& msg) {
    last_drone_command_dyaw_msg = msg;
    drone_EKF_state_estimator.setDroneCommands( last_drone_command_pitch_roll_msg.pitchCmd,
                                                last_drone_command_pitch_roll_msg.rollCmd,
                                                last_drone_command_dyaw_msg.dYawCmd,
                                                last_drone_command_daltitude_msg.dAltitudeCmd);
    return;
}

void DroneEKFStateEstimatorROSModule::droneDroneCommandDAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd& msg) {
    last_drone_command_daltitude_msg = msg;
    drone_EKF_state_estimator.setDroneCommands( last_drone_command_pitch_roll_msg.pitchCmd,
                                                last_drone_command_pitch_roll_msg.rollCmd,
                                                last_drone_command_dyaw_msg.dYawCmd,
                                                last_drone_command_daltitude_msg.dAltitudeCmd);
    return;
}

void DroneEKFStateEstimatorROSModule::close() {
    DroneModule::close();
}

bool DroneEKFStateEstimatorROSModule::run() {
    DroneModule::run();
    if (!moduleStarted)
        return false;

    drone_EKF_state_estimator.stateEstimation(ekf_mahalanobis_distance);

    // Get and publish: dronePose and droneSpeed; on GMRwrtGFF and LMrTwrtEKF
    getEstimatedPoses_FromEKF();
    getEstimatedSpeeds_FromEKF();
    publishEstimatedPose();
    publishEstimatedSpeeds();

    logEKFState();
    drone_logger_ros_publisher.run();

    return true;
}

int DroneEKFStateEstimatorROSModule::getEstimatedPoses_FromEKF(void) {
    double actual_time = ros::Time::now().toSec();
    last_drone_estimated_LMrTwrtEKF_pose_msg.time = actual_time;
    last_drone_estimated_LMrTwrtEKF_pose_msg.YPR_system      = "wYvPuR";
    last_drone_estimated_LMrTwrtEKF_pose_msg.target_frame    = "drone_LMrT";
    last_drone_estimated_LMrTwrtEKF_pose_msg.reference_frame = "EKF";
    drone_EKF_state_estimator.getDroneEstimatedPose_LMrTwrtEKF(
        &(last_drone_estimated_LMrTwrtEKF_pose_msg.x),
        &(last_drone_estimated_LMrTwrtEKF_pose_msg.y),
        &(last_drone_estimated_LMrTwrtEKF_pose_msg.z),
        &(last_drone_estimated_LMrTwrtEKF_pose_msg.yaw),
        &(last_drone_estimated_LMrTwrtEKF_pose_msg.pitch),
        &(last_drone_estimated_LMrTwrtEKF_pose_msg.roll));
    last_drone_estimated_GMRwrtGFF_pose_msg.time = actual_time;
    last_drone_estimated_GMRwrtGFF_pose_msg.YPR_system      = "wYvPuR";
    last_drone_estimated_GMRwrtGFF_pose_msg.target_frame    = "drone_GMR";
    last_drone_estimated_GMRwrtGFF_pose_msg.reference_frame = "GFF";
    drone_EKF_state_estimator.getDroneEstimatedPose_GMRwrtGFF(
        &(last_drone_estimated_GMRwrtGFF_pose_msg.x),
        &(last_drone_estimated_GMRwrtGFF_pose_msg.y),
        &(last_drone_estimated_GMRwrtGFF_pose_msg.z),
        &(last_drone_estimated_GMRwrtGFF_pose_msg.yaw),
        &(last_drone_estimated_GMRwrtGFF_pose_msg.pitch),
        &(last_drone_estimated_GMRwrtGFF_pose_msg.roll));
    return 1;
}

int DroneEKFStateEstimatorROSModule::getEstimatedSpeeds_FromEKF() {
    double actual_time = ros::Time::now().toSec();
    last_drone_estimated_LMrTwrtEKF_speeds_msg.time   = actual_time;
//    last_drone_estimated_LMrTwrtEKF_speeds_msg.YPR_system      = "wYvPuR";
//    last_drone_estimated_LMrTwrtEKF_speeds_msg.target_frame    = "drone_LMrT";
//    last_drone_estimated_LMrTwrtEKF_speeds_msg.reference_frame = "EKF";
    drone_EKF_state_estimator.getDroneEstimatedSpeed_LMrTwrtEKF(
        &(last_drone_estimated_LMrTwrtEKF_speeds_msg.dx),
        &(last_drone_estimated_LMrTwrtEKF_speeds_msg.dy),
        &(last_drone_estimated_LMrTwrtEKF_speeds_msg.dz),
        &(last_drone_estimated_LMrTwrtEKF_speeds_msg.dyaw),
        &(last_drone_estimated_LMrTwrtEKF_speeds_msg.dpitch),
        &(last_drone_estimated_LMrTwrtEKF_speeds_msg.droll));
    last_drone_estimated_GMRwrtGFF_speeds_msg.time = actual_time;
//    last_drone_estimated_GMRwrtGFF_speeds_msg.YPR_system      = "wYvPuR";
//    last_drone_estimated_GMRwrtGFF_speeds_msg.target_frame    = "drone_GMR";
//    last_drone_estimated_GMRwrtGFF_speeds_msg.reference_frame = "GFF";
    drone_EKF_state_estimator.getDroneEstimatedSpeed_GMRwrtGFF(
        &(last_drone_estimated_GMRwrtGFF_speeds_msg.dx),
        &(last_drone_estimated_GMRwrtGFF_speeds_msg.dy),
        &(last_drone_estimated_GMRwrtGFF_speeds_msg.dz),
        &(last_drone_estimated_GMRwrtGFF_speeds_msg.dyaw),
        &(last_drone_estimated_GMRwrtGFF_speeds_msg.dpitch),
        &(last_drone_estimated_GMRwrtGFF_speeds_msg.droll));
    return 1;
}

//void DroneEKFStateEstimatorROSModule::init() {
//}

bool DroneEKFStateEstimatorROSModule::resetValues() {
    drone_EKF_state_estimator.resetStateEstimator();

    // Get and publish: dronePose and droneSpeed; on GMRwrtGFF and LMrTwrtEKF
    getEstimatedPoses_FromEKF();
    getEstimatedSpeeds_FromEKF();
    publishEstimatedPose();
    publishEstimatedSpeeds();
    return DroneModule::resetValues();
}

bool DroneEKFStateEstimatorROSModule::startVal() {
    drone_EKF_state_estimator.startStateEstimator();

    // Get and publish: dronePose and droneSpeed; on GMRwrtGFF and LMrTwrtEKF
    getEstimatedPoses_FromEKF();
    getEstimatedSpeeds_FromEKF();
    publishEstimatedPose();
    publishEstimatedSpeeds();
    return DroneModule::startVal();
}

bool DroneEKFStateEstimatorROSModule::stopVal() {
    return DroneModule::stopVal();
}

bool DroneEKFStateEstimatorROSModule::setInitDroneYaw(droneMsgsROS::setInitDroneYaw_srv_type::Request& request, droneMsgsROS::setInitDroneYaw_srv_type::Response& response) {
    double yaw_droneLMrT_telemetry_rad = request.yaw_droneLMrT_telemetry_rad;
    drone_EKF_state_estimator.setInitDroneYaw(yaw_droneLMrT_telemetry_rad);
    response.ack = true;

    debug_string_stacker
                            << " [ekf;set_init_yaw]"
                            << " yaw_rad:"     << yaw_droneLMrT_telemetry_rad
                            << std::endl;
    debug_string_stacker.setPriorityFlag();

    return response.ack;
}

void DroneEKFStateEstimatorROSModule::logEKFState() {
    debug_string_stacker
//              /* timestamp   NOTE: done in drone logger */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
        /* tag         */   << "[ekf;state]"
        /* isStarted   */   << " started:" << isStarted()
        /* xs, ys, zs               */  << " xs:"     << last_drone_estimated_GMRwrtGFF_pose_msg.x
        /* yaws, pitchs, rolls      */  << " ys:"     << last_drone_estimated_GMRwrtGFF_pose_msg.y
                                        << " zs:"     << last_drone_estimated_GMRwrtGFF_pose_msg.z
                                        << " yaws:"   << last_drone_estimated_GMRwrtGFF_pose_msg.yaw
                                        << " pitchs:" << last_drone_estimated_GMRwrtGFF_pose_msg.pitch
                                        << " rolls:"  << last_drone_estimated_GMRwrtGFF_pose_msg.roll
        /* vxs, vys, vzs            */  << " vxs:"    << last_drone_estimated_GMRwrtGFF_speeds_msg.dx
        /* dyaws                    */  << " vys:"    << last_drone_estimated_GMRwrtGFF_speeds_msg.dy
                                        << " vzs:"    << last_drone_estimated_GMRwrtGFF_speeds_msg.dz
                                        << " dyaws:"  << last_drone_estimated_GMRwrtGFF_speeds_msg.dyaw
                                        << std::endl;
}

