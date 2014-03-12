//////////////////////////////////////////////////////
//  parrotARDroneOuts.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Dec 23, 2013
//      Author: joselusl
//
//  Description: file to manage ardrone_autonomy node
//  See: https://github.com/AutonomyLab/ardrone_autonomy
//
//////////////////////////////////////////////////////


#ifndef _PARROT_ARDRONE_OUTS_H
#define _PARROT_ARDRONE_OUTS_H


#include "droneModuleROS.h"
#include "communication_definition.h"



//ARDrone autonomy Navdata
#include "ardrone_autonomy/Navdata.h"


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




/////// OpenCV ///////
//#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
//#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
//#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur




//// ROS  ///////
#include "ros/ros.h"



///// AR DRONE AUTONOMY /////
////Services
//#include "std_srvs/Empty.h"

//#include "ardrone_autonomy/CamSelect.h"
//#include "ardrone_autonomy/LedAnim.h"
//#include "ardrone_autonomy/FlightAnim.h"

////Topics
//#include "std_msgs/Empty.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/String.h"

////images
//#include <cv_bridge/cv_bridge.h>



//#include <sensor_msgs/image_encodings.h>




//#include <geometry_msgs/Twist.h>




//I/O stream
//std::cout
#include <iostream>

//Math
//M_PI
#include <cmath>




/////////////////////////////////////////
// Class Imu
//
//   Description
//
/////////////////////////////////////////
class ImuROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher ImuPubl;
    bool publishImuValue();


    //Subscriber
protected:
    ros::Subscriber ImuSubs;
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);


    //Imu msgs
protected:
    sensor_msgs::Imu ImuMsgs;


    //Constructors and destructors
public:
    ImuROSModule();
    ~ImuROSModule();


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





/////////////////////////////////////////
// Class Temperature
//
//   Description: Needs to be adjusted the unit!
//
/////////////////////////////////////////
class TemperatureROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher TemperaturePubl;
    bool publishTemperatureValue();


    //Subscriber
protected:
    ros::Subscriber TemperatureSubs;
    void temperatureCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //Temperature msgs
protected:
    sensor_msgs::Temperature TemperatureMsgs;


    //Constructors and destructors
public:
    TemperatureROSModule();
    ~TemperatureROSModule();


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



/////////////////////////////////////////
// Class Magnetometer
//
//   Description
//
/////////////////////////////////////////
class MagnetometerROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher MagnetometerPubl;
    bool publishMagnetometerValue();


    //Subscriber
protected:
    ros::Subscriber MagnetometerSubs;
    void magnetometerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);


    //Magnetometer msgs
protected:
    geometry_msgs::Vector3Stamped MagnetometerMsgs;


    //Constructors and destructors
public:
    MagnetometerROSModule();
    ~MagnetometerROSModule();


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



/////////////////////////////////////////
// Class Battery
//
//   Description: Needs to be adjusted the unit!
//
/////////////////////////////////////////
class BatteryROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher BatteryPubl;
    bool publishBatteryValue();


    //Subscriber
protected:
    ros::Subscriber BatterySubs;
    void batteryCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //Battery msgs
protected:
    droneMsgsROS::battery BatteryMsgs;


    //Constructors and destructors
public:
    BatteryROSModule();
    ~BatteryROSModule();


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




/////////////////////////////////////////
// Class Altitude
//
//   Description
//
/////////////////////////////////////////
class AltitudeROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher AltitudePubl;
    bool publishAltitudeValue();


    //Subscriber
protected:
    ros::Subscriber AltitudeSubs;
    void altitudeCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //Altitude msgs
protected:
    droneMsgsROS::droneAltitude AltitudeMsgs;


    //Constructors and destructors
public:
    AltitudeROSModule();
    ~AltitudeROSModule();


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


/////////////////////////////////////////
// Class RotationAngles
//
//   Description
//
/////////////////////////////////////////
class RotationAnglesROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher RotationAnglesPubl;
    bool publishRotationAnglesValue();


    //Subscriber
protected:
    ros::Subscriber RotationAnglesSubs;
    void rotationAnglesCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //RotationAngles msgs
protected:
    geometry_msgs::Vector3Stamped RotationAnglesMsgs;


    //Constructors and destructors
public:
    RotationAnglesROSModule();
    ~RotationAnglesROSModule();


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



/////////////////////////////////////////
// Class GroundSpeed
//
//   Description:
//
/////////////////////////////////////////
class GroundSpeedROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher GroundSpeedPubl;
    bool publishGroundSpeedValue();


    //Subscriber
protected:
    ros::Subscriber GroundSpeedSubs;
    void groundSpeedCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //GroundSpeed msgs
protected:
    droneMsgsROS::vector2Stamped GroundSpeedMsgs;


    //Constructors and destructors
public:
    GroundSpeedROSModule();
    ~GroundSpeedROSModule();


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


/////////////////////////////////////////
// Class Pressure
//
//   Description: Needs to be adjusted the unit!
//
/////////////////////////////////////////
class PressureROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher PressurePubl;
    bool publishPressureValue();


    //Subscriber
protected:
    ros::Subscriber PressureSubs;
    void pressureCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //Pressure msgs
protected:
    sensor_msgs::FluidPressure PressureMsgs;


    //Constructors and destructors
public:
    PressureROSModule();
    ~PressureROSModule();


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



/////////////////////////////////////////
// Class DroneStatus
//
//   Description: Needs to be adjusted the unit!
//
/////////////////////////////////////////
class DroneStatusROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher DroneStatusPubl;
    bool publishDroneStatusValue();


    //Subscriber
protected:
    ros::Subscriber DroneStatusSubs;
    void droneStatusCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);


    //DroneStatus msgs
protected:
    droneMsgsROS::droneStatus DroneStatusMsgs;


    //Constructors and destructors
public:
    DroneStatusROSModule();
    ~DroneStatusROSModule();


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



/////////////////////////////////////////
// Class BottomCamera
//
//   Description:
//
/////////////////////////////////////////
class BottomCameraROSModule : public DroneModule
{
    //Publisher
protected:
    image_transport::CameraPublisher BottomCameraPubl;
    bool publishBottomCameraValue();


    //Subscriber
protected:
    image_transport::CameraSubscriber BottomCameraSubs;
    void bottomCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg);


    //BottomCamera msgs
protected:
    sensor_msgs::ImageConstPtr BottomCameraMsgs;
    sensor_msgs::CameraInfoConstPtr BottomCameraInfoMsgs;


    //Constructors and destructors
public:
    BottomCameraROSModule();
    ~BottomCameraROSModule();


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



/////////////////////////////////////////
// Class FrontCamera
//
//   Description:
//
/////////////////////////////////////////
class FrontCameraROSModule : public DroneModule
{
    //Publisher
protected:
    image_transport::CameraPublisher FrontCameraPubl;
    bool publishFrontCameraValue();


    //Subscriber
protected:
    image_transport::CameraSubscriber FrontCameraSubs;
    void frontCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg);


    //FrontCamera msgs
protected:
    sensor_msgs::ImageConstPtr FrontCameraMsgs;
    sensor_msgs::CameraInfoConstPtr FrontCameraInfoMsgs;


    //Constructors and destructors
public:
    FrontCameraROSModule();
    ~FrontCameraROSModule();


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
