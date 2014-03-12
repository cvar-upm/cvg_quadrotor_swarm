//////////////////////////////////////////////////////
//  droneInterfaceROSModule.h
//
//  Created on: Dec 19, 2013
//      Author: joselusl
//
//  Last modification on: Dec 23, 2013
//      Author: joselusl
//
//  Description: class to create an interface
//
//////////////////////////////////////////////////////


#ifndef _DRONE_INTERFACE_ROS_MODULE_H
#define _DRONE_INTERFACE_ROS_MODULE_H



//I/O stream
//std::cout
#include <iostream>
#include <string>

//Math
//M_PI
#include <cmath>


//// ROS  ///////
#include "ros/ros.h"


//IMU
#include "sensor_msgs/Imu.h"

//Temperature
#include "sensor_msgs/Temperature.h"

//Magnetometer and RotationAngles
#include "geometry_msgs/Vector3Stamped.h"

//Battery
#include "droneMsgsROS/battery.h"

//Altitude
#include "droneMsgsROS/droneAltitude.h"

//Ground Speed
#include "droneMsgsROS/vector2Stamped.h"

//Pressure
#include "sensor_msgs/FluidPressure.h"

//Drone Status
#include "droneMsgsROS/droneStatus.h"

//BottomCamera and FrontCamera
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/////// OpenCV ///////
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)





//Custom commands
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"

#include "droneMsgsROS/droneCommand.h"





#include "droneModuleROS.h"
#include "communication_definition.h"




#define FREQ_INTERFACE  200.0

class DroneInterfaceROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher DronePitchRollCmdPubl;
    ros::Publisher DroneDAltitudeCmdPubl;
    ros::Publisher DroneDYawCmdPubl;
    ros::Publisher DroneCommandPubl;

public:
    bool publishPitchRollCmd();
    bool publishDAltitudeCmd();
    bool publishDYawCmd();
    bool publishDroneCmd();


    //Subscriber
protected:
    ros::Subscriber ImuSubs;
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::Subscriber TemperatureSubs;
    void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& msg);

    ros::Subscriber MagnetometerSubs;
    void magnetometerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    ros::Subscriber BatterySubs;
    void batteryCallback(const droneMsgsROS::battery::ConstPtr& msg);

    ros::Subscriber AltitudeSubs;
    void altitudeCallback(const droneMsgsROS::droneAltitude::ConstPtr& msg);

    ros::Subscriber RotationAnglesSubs;
    void rotationAnglesCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    ros::Subscriber GroundSpeedSubs;
    void groundSpeedCallback(const droneMsgsROS::vector2Stamped::ConstPtr& msg);

    ros::Subscriber PressureSubs;
    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);

    ros::Subscriber DroneStatusSubs;
    void droneStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg);

    ros::Subscriber BottomCameraSubs;
    void bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Subscriber FrontCameraSubs;
    void frontCameraCallback(const sensor_msgs::ImageConstPtr& msg);


    //Command
    ros::Subscriber DronePitchRollCmdSubs;
    void dronePitchRollCmdCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg);

    ros::Subscriber DroneDAltitudeCmdSubs;
    void droneDAltitudeCmdCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg);

    ros::Subscriber DroneDYawCmdSubs;
    void droneDYawCmdCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg);

    ros::Subscriber DroneCommandSubs;
    void droneCommandCallback(const droneMsgsROS::droneCommand::ConstPtr& msg);


    //Msgs
public:
    sensor_msgs::Imu ImuMsgs;
    sensor_msgs::Temperature TemperatureMsgs;
    geometry_msgs::Vector3Stamped MagnetometerMsgs;
    droneMsgsROS::battery BatteryMsgs;
    droneMsgsROS::droneAltitude AltitudeMsgs;
    geometry_msgs::Vector3Stamped RotationAnglesMsgs;
    droneMsgsROS::vector2Stamped GroundSpeedMsgs;
    sensor_msgs::FluidPressure PressureMsgs;
    droneMsgsROS::droneStatus DroneStatusMsgs;

protected:
    cv_bridge::CvImagePtr cvBottomImage;
public:
    bool newBottomImage;
    cv::Mat bottomImage;
protected:
    cv_bridge::CvImagePtr cvFrontImage;
public:
    bool newFrontImage;
    cv::Mat frontImage;

public:
    droneMsgsROS::dronePitchRollCmd DronePitchRollCmdMsgs;
    droneMsgsROS::droneDAltitudeCmd DroneDAltitudeCmdMsgs;
    droneMsgsROS::droneDYawCmd DroneDYawCmdMsgs;
    droneMsgsROS::droneCommand DroneCommandMsgs;




    //Constructors and destructors
public:
    DroneInterfaceROSModule();
    ~DroneInterfaceROSModule();


    //Init and close
public:
    void init();
    void close();

    //Open
 public:
    void open(ros::NodeHandle & nIn, std::string moduleName);

    //Reset
protected:
    bool resetValues();

    //Start
protected:
    bool startVal();

    //Stop
protected:
    bool stopVal();

    //Run
public:
    bool run();


public:
    void clearCmd();

public: // interface_jp
    void drone_take_off();
    void drone_force_take_off();
    void drone_land();
    void drone_hover();
    void drone_emergency_stop();
    void drone_reset();
    void drone_move();
    std::stringstream *getOdometryStream();
    std::stringstream *getDroneCommandsStream();
private:
    std::stringstream interface_printout_stream;
};






#endif
