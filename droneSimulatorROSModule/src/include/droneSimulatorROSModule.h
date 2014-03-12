//////////////////////////////////////////////////////
//  droneSimulatorROSModule.h
//
//  Created on: Jan 22, 2014
//      Author: joselusl
//
//  Last modification on: Jan 22, 2014
//      Author: joselusl
//
//  Description: file to manage ardrone_autonomy node
//  See: https://github.com/AutonomyLab/ardrone_autonomy
//
//////////////////////////////////////////////////////


#ifndef _DRONE_SIMULATOR_H
#define _DRONE_SIMULATOR_H


#include "droneModuleROS.h"


#include "communication_definition.h"



//// ROS  ///////
#include "ros/ros.h"



//Custom commands
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"

#include "droneMsgsROS/droneCommand.h"


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



//Drone pose
#include "droneMsgsROS/dronePoseStamped.h"



//I/O stream
//std::cout
#include <iostream>

//Math
//M_PI
#include <cmath>



//DroneSimulator
#include "droneSimulator.h"

//HL commands
#include "drone_utils/drone_state_command_enum.h"
//Drone Status
#include "drone_utils/drone_state_enum.h"


///Consts
const double DRONE_SIMULATOR_RATE = 200.0;

/////////////////////////////////////////
// Class DroneSimulator
//
//   Description
//
/////////////////////////////////////////
class DroneSimulatorROSModule : public DroneModule
{
    //Simulator
protected:
    DroneSimulator MyDroneSimulator;



    //Subscribers
protected:
    ros::Subscriber PitchRollSubs;
    void pitchRollCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg);

    ros::Subscriber AltitudeSubs;
    void dAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg);

    ros::Subscriber YawSubs;
    void dYawCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg);

    ros::Subscriber CommandSubs;
    void commandCallback(const droneMsgsROS::droneCommand::ConstPtr& msg);



    //Publishers
protected:
    ros::Publisher ImuPubl;
    sensor_msgs::Imu ImuMsgs;
    bool publishImuValue();

    ros::Publisher TemperaturePubl;
    sensor_msgs::Temperature TemperatureMsgs;
    bool publishTemperatureValue();

    ros::Publisher MagnetometerPubl;
    geometry_msgs::Vector3Stamped MagnetometerMsgs;
    bool publishMagnetometerValue();

    ros::Publisher BatteryPubl;
    droneMsgsROS::battery BatteryMsgs;
    bool publishBatteryValue();

    ros::Publisher AltitudePubl;
    droneMsgsROS::droneAltitude AltitudeMsgs;
    bool publishAltitudeValue();

    ros::Publisher RotationAnglesPubl;
    geometry_msgs::Vector3Stamped RotationAnglesMsgs;
    bool publishRotationAnglesValue();

    ros::Publisher GroundSpeedPubl;
    droneMsgsROS::vector2Stamped GroundSpeedMsgs;
    bool publishGroundSpeedValue();

    ros::Publisher PressurePubl;
    sensor_msgs::FluidPressure PressureMsgs;
    bool publishPressureValue();

    ros::Publisher DroneStatusPubl;
    droneMsgsROS::droneStatus DroneStatusMsgs;
    bool publishDroneStatusValue();



    //Internal Pose publisher
protected:
    ros::Publisher InternalDronePosePubl;
    droneMsgsROS::dronePoseStamped InternalDronePoseMsgs;
    bool publishInternalDronePose();


    //Constructors and destructors
public:
    DroneSimulatorROSModule();
    ~DroneSimulatorROSModule();


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


};



#endif
