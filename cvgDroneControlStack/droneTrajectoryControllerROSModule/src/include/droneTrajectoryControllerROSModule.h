#ifndef DRONETRAJECTORYCONTROLLERROSMODULE_H
#define DRONETRAJECTORYCONTROLLERROSMODULE_H

#include "ros/ros.h"
#include "droneModuleROS.h"
#include "communication_definition.h"
#include "droneTrajectoryController.h"
//Drone msgs
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/dronePositionRefCommandStamped.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"
#include "droneMsgsROS/droneYawRefCommand.h"
#include "droneMsgsROS/droneTrajectoryControllerControlMode.h"
//Services
//#include "droneTrajectoryControllerROSModule/setControlMode.h"
#include "droneMsgsROS/setControlMode.h"
#include "droneloggerrospublisher.h"

class DroneTrajectoryControllerROSModule : public DroneModule {
public:
    DroneTrajectoryControllerROSModule();
    ~DroneTrajectoryControllerROSModule();

    // DroneModule stuff: reset, start, stop, run, init, open, close
private:
    void init();
    void close();
protected:
    bool resetValues();
    bool startVal();
    bool stopVal();
public:
    bool run();
    void open(ros::NodeHandle & nIn, std::string moduleName);

private:
    DroneTrajectoryController drone_trajectory_controller;
    bool setControlModeVal(Controller_MidLevel_controlMode::controlMode mode);

    // [Subscribers] Controller references
private:
    ros::Subscriber dronePositionRefSub;
    void dronePositionRefsSubCallback(const droneMsgsROS::dronePositionRefCommandStamped::ConstPtr &msg);
    ros::Subscriber droneSpeedsRefSub;
    void droneSpeedsRefsSubCallback(const droneMsgsROS::droneSpeeds::ConstPtr &msg);
    ros::Subscriber droneTrajectoryAbsRefSub;  // Trajectory specified in world coordinates
    void droneTrajectoryAbsRefCommandCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand::ConstPtr& msg);
    ros::Subscriber droneTrajectoryRelRefSub;  // Trajectory specified relative to current drone pose
    void droneTrajectoryRelRefCommandCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand::ConstPtr& msg);
    bool stay_in_position;
    ros::Subscriber droneYawRefCommandSub;
    void droneYawRefCommandCallback(const droneMsgsROS::droneYawRefCommand::ConstPtr& msg);

    // [Publishers] Controller references, meaning of useful values depend on the current control mode
    ros::Publisher drone_position_reference_publisher;
    droneMsgsROS::dronePose   current_drone_position_reference;                      // useful values: x, y, z, yaw
    ros::Publisher drone_speed_reference_publisher;
    droneMsgsROS::droneSpeeds current_drone_speed_reference;                         // useful values: vxfi, vyfi, vzfi (dyawfi is unused, undebugged)
    ros::Publisher drone_trajectory_reference_publisher;
    droneMsgsROS::dronePositionTrajectoryRefCommand current_drone_trajectory_command;// useful values: current trajectory: initial_checkpoint,isPeridioc, checkpoints[]
    void publishControllerReferences();

    // [Subscribers] State estimation: controller feedback inputs
private:
    ros::Subscriber droneEstimatedPoseSubs;
    void droneEstimatedPoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);
    droneMsgsROS::dronePose last_estimatedPose;
    ros::Subscriber droneEstimatedSpeedsSubs;
    void droneEstimatedSpeedsCallback(const droneMsgsROS::droneSpeeds::ConstPtr& msg);
    droneMsgsROS::droneSpeeds last_estimatedSpeed;

    // Publishers
private:
    ros::Publisher controlModePub;
    void publishControlMode();
    ros::Publisher drone_command_pitch_roll_publisher;
    ros::Publisher drone_command_dyaw_publisher;
    ros::Publisher drone_command_daltitude_publisher;
    droneMsgsROS::dronePitchRollCmd drone_command_pitch_roll_msg;
    droneMsgsROS::droneDAltitudeCmd drone_command_daltitude_msg;
    droneMsgsROS::droneDYawCmd      drone_command_dyaw_msg;
    int publishDroneNavCommand(void);
    void setNavCommand(float pitch, float roll, float dyaw, float dz, double time=-1.0);
    void setNavCommandToZero();


    // Service servers
private:
    ros::ServiceServer setControlModeServerSrv;
    bool setControlModeServCall(droneMsgsROS::setControlMode::Request& request, droneMsgsROS::setControlMode::Response& response);

    // Drone Logger
    DroneLoggerROSPublisher drone_logger_ros_publisher;
};

#endif // DRONETRAJECTORYCONTROLLERROSMODULE_H
