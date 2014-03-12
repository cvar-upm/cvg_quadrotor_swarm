#ifndef DRONEEKFSTATEESTIMATORROSMODULE_H
#define DRONEEKFSTATEESTIMATORROSMODULE_H

#include "ros/ros.h"
#include "droneModuleROS.h"
#include "communication_definition.h"
#include "droneEKFStateEstimator.h"
//Drone msgs
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "droneMsgsROS/droneNavCommand.h" //Input to the process model for prediction
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/droneNavData.h"
#include "droneMsgsROS/droneSensorData.h"
#include "droneMsgsROS/droneAltitude.h"
#include "droneMsgsROS/vector2Stamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Vector3Stamped.h"
//services
#include "droneMsgsROS/setInitDroneYaw_srv_type.h"
#include "debugstringstacker.h"
#include "droneloggerrospublisher.h"

class DroneEKFStateEstimatorROSModule : public DroneModule {
public:
    DroneEKFStateEstimatorROSModule();
    ~DroneEKFStateEstimatorROSModule();

private:
    DroneEKFStateEstimator drone_EKF_state_estimator;
    float ekf_mahalanobis_distance;

    // Subscribers
private:
    ros::Subscriber drone_rotation_angles_subscriber;
    ros::Subscriber drone_altitude_subscriber;
    ros::Subscriber drone_ground_optical_flow_subscriber;
    geometry_msgs::Vector3Stamped last_rotation_angles_msg;
    droneMsgsROS::droneAltitude last_altitude_msg;
    droneMsgsROS::vector2Stamped last_ground_optical_flow_msg;
    void droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg);
    void droneAltitudeCallback(const droneMsgsROS::droneAltitude& msg);
    void droneGroundOpticalFlowCallback(const droneMsgsROS::vector2Stamped& msg);


    ros::Subscriber drone_command_pitch_roll_subscriber;
    ros::Subscriber drone_commdan_dyaw_subscriber;
    ros::Subscriber drone_command_daltitude_subscriber;
    droneMsgsROS::dronePitchRollCmd last_drone_command_pitch_roll_msg;
    droneMsgsROS::droneDYawCmd      last_drone_command_dyaw_msg;
    droneMsgsROS::droneDAltitudeCmd last_drone_command_daltitude_msg;
    void droneDroneCommandPitchRollCallback(const droneMsgsROS::dronePitchRollCmd& msg);
    void droneDroneCommandDYawCallback(const droneMsgsROS::droneDYawCmd& msg);
    void droneDroneCommandDAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd& msg);

    // Publishers
private:
    ros::Publisher drone_estimated_LMrT_pose_publisher;
    ros::Publisher drone_estimated_GMR_pose_publisher;
    ros::Publisher drone_estimated_LMrT_speeds_publisher;
    ros::Publisher drone_estimated_GMR_speeds_publisher;
    droneMsgsROS::dronePose last_drone_estimated_LMrTwrtEKF_pose_msg;
    droneMsgsROS::dronePose last_drone_estimated_GMRwrtGFF_pose_msg;
    droneMsgsROS::droneSpeeds last_drone_estimated_LMrTwrtEKF_speeds_msg;
    droneMsgsROS::droneSpeeds last_drone_estimated_GMRwrtGFF_speeds_msg;
    int getEstimatedPoses_FromEKF();
    int getEstimatedSpeeds_FromEKF();
    int publishEstimatedPose();
    int publishEstimatedSpeeds();

    // Transformation between different reference frames, see documentation for more information
    // Service servers
private:
    ros::ServiceServer   setDroneYawInitSrv;
    bool setInitDroneYaw(droneMsgsROS::setInitDroneYaw_srv_type::Request& request, droneMsgsROS::setInitDroneYaw_srv_type::Response& response);

public:
    void open(ros::NodeHandle &nIn, std::string moduleName);
    void close();
    bool run();
protected:
//    void init();

    //Reset, start, and stop State Estimator
protected:
    bool resetValues();
    bool startVal();
    bool stopVal();

    // DroneLogger - Logging functions
    DroneLoggerROSPublisher drone_logger_ros_publisher;
    DebugStringStacker debug_string_stacker;
public:
    void logEKFState();              // called from ROSModule after getOutput(...) call
};

#endif // DRONEEKFSTATEESTIMATORROSMODULE_H
