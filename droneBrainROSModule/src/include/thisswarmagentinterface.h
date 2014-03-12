#ifndef THISSWARMAGENTINTERFACE_H
#define THISSWARMAGENTINTERFACE_H

// ROS
#include "ros/ros.h"

// C++ standar libraries
#include <cstdlib>
#include <stdio.h>
#include <iostream>

// Topic and Rosservice names
#include "communication_definition.h"

#include "dronemoduleinterface.h"
#include "droneMsgsROS/droneNavData.h"
#include "droneMsgsROS/dronePose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

//#include "ardrone_autonomy/Navdata.h"
// Navdata is substituted by:
//  [*] bateria
//  [*] droneStatus
//  [*] altitud
//  [*] groundSpeed
//  [*] imuNode
//  magnetometerNode
//  pressureNode
//  [?] rotationAngles
//  temperatureNode
#include "communication_definition.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <droneMsgsROS/droneAltitude.h>
#include <droneMsgsROS/vector2Stamped.h>
#include <droneMsgsROS/battery.h>
#include <droneMsgsROS/droneStatus.h>
#include "droneMsgsROS/droneCommand.h"

#define SWARM_AGENT_WIFI_TIMEOUT_THRESHOLD (2.0)
#define SWARM_AGENT_BATTERY_LEVEL_CHECK_THRESHOLD (25.0)

class ThisSwarmAgentInterface {
protected:
    ros::NodeHandle      n;
public:
    DroneModuleInterface state_estimator;
    DroneModuleInterface trajectory_controller;
    DroneModuleInterface arucoeye;
    DroneModuleInterface localizer;
    DroneModuleInterface obstacle_processor;
    DroneModuleInterface trajectory_planner;
    DroneModuleInterface yaw_planner;
    DroneModuleInterface mission_planner;
private:
//    ros::Subscriber navdataSub;
    ros::Time last_navdata_timestamp;
//    void navdataSubCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);
    bool wifi_is_ok;
    ros::Subscriber estimatedPoseSub;
    void estimatedPoseSubCallback(const droneMsgsROS::dronePose::ConstPtr& msg);
public: // please, use as read only...
//    ardrone_autonomy::Navdata last_navdata;         // To have battery state, etc
//    droneMsgsROS::droneNavData   last_droneNavData;    // To have navdata in MAVwork reference frame
    droneMsgsROS::dronePose last_estimatedPose;        // to have this_drone last estimated pose (from localizer)

private:  // Subscribers
    ros::Subscriber drone_rotation_angles_subscriber;
    ros::Subscriber drone_altitude_subscriber;
    ros::Subscriber drone_ground_optical_flow_subscriber;
    ros::Subscriber battery_level_subscriber;
    ros::Subscriber drone_status_subscriber;
public: // please, use as read only...
    geometry_msgs::Vector3Stamped last_rotation_angles_msg;
    droneMsgsROS::droneAltitude last_altitude_msg;
    droneMsgsROS::vector2Stamped last_ground_optical_flow_msg;
    droneMsgsROS::battery last_battery_msg;
    droneMsgsROS::droneStatus last_drone_status_msg;
private:
    void droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg);
    void droneAltitudeCallback(const droneMsgsROS::droneAltitude& msg);
    void droneGroundOpticalFlowCallback(const droneMsgsROS::vector2Stamped& msg);
    void batteryCallback(const droneMsgsROS::battery::ConstPtr& msg);
    void droneStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg);


public:
    ThisSwarmAgentInterface();
    ~ThisSwarmAgentInterface();
    void open(ros::NodeHandle & nIn);
    bool startModule( ModuleNames::name module_name_enum);
    bool stopModule( ModuleNames::name module_name_enum);
    bool resetModule( ModuleNames::name module_name_enum);
    bool isStartedModule( ModuleNames::name module_name_enum);
    bool isWifiOk();
    bool batteryCheckIsOk();
    float battery_threshold;

    // Parrot/drone flying-mode commands
private:
//    geometry_msgs::Twist navCommand;
//    std_msgs::Empty emptyMsg;
//    ros::Publisher takeOffPubl;
//    ros::Publisher landPubl;
//    ros::Publisher resetPubl;
//    ros::Publisher commandsPubl;
    ros::Publisher DroneCommandPubl;


public:
    void drone_takeOff();
    void drone_land();
    void drone_reset();
    void drone_hover();
    void drone_move();
};

#endif // THISSWARMAGENTINTERFACE_H
