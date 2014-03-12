#ifndef _DRONE_ELEMENTS_H
#define _DRONE_ELEMENTS_H



//HL commands
//#include "drone_utils/drone_state_command_enum.h"


//I/O stream
//std::cout
#include <iostream>

//Math
//
#include <cmath>


//White noise
#include <random>

//AltitudeSensor
const double NOISE_MEAN_AltitudeSensor=0.0;
const double NOISE_STD_AltitudeSensor=0.03;
//RotationAnglesSensor
const double NOISE_MEAN_RotationAnglesSensor=0.0;
const double NOISE_STD_RotationAnglesSensor=1.0;
//GroundSpeedSensor
const double NOISE_MEAN_GroundSpeedSensor=0.0;
const double NOISE_STD_GroundSpeedSensor=0.05;

//const double NOISE_MEAN=0.0;
//const double NOISE_STD=0.25;



/////////////////////////////////////////
// Class Battery
//
//   Description
//
/////////////////////////////////////////
class Battery
{
protected:
    double percentaje;

public:
    Battery();
    ~Battery();

public:
    double getPercentaje();
    int setPercenaje(double percentajeIn);
};


/////////////////////////////////////////
// Class IMU
//
//   Description: JL TODO
//
/////////////////////////////////////////
class IMU
{
protected:


public:
    IMU();
    ~IMU();

public:

};


/////////////////////////////////////////
// Class Termometer
//
//   Description
//
/////////////////////////////////////////
class Termometer
{
protected:
    double temperature;
    double varTemperature;


public:
    Termometer();
    ~Termometer();

public:
    int setTemperature(double temperatureIn, double varTemperatureIn=0.0);
    int getTemperature(double& temperatureOut, double& varTemperatureOut);

};


/////////////////////////////////////////
// Class Magnetometer
//
//   Description
//
/////////////////////////////////////////
class Magnetometer
{
protected:
    double magX;
    double magY;
    double magZ;


public:
    Magnetometer();
    ~Magnetometer();

public:
    int setMagnetometer(double magXIn, double magYIn, double magZIn);
    int getMagnetometer(double& magXOut, double& magYOut, double& magZOut);



};


/////////////////////////////////////////
// Class AltitudeSensor
//
//   Description
//
/////////////////////////////////////////
class AltitudeSensor
{
protected:
    double altitude;
    double altitudeSpeed;

    double varAltitude;
    double varAltitudeSpeed;


    //White noise
protected:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

public:
    AltitudeSensor();
    ~AltitudeSensor();

public:
    int setAltitude(double altitudeIn, double varAltitudeIn=0.0);
    int getAltitude(double& altitudeOut, double& varAltitudeOut);
    int setAltitudeSpeed(double altitudeSpeedIn, double varAltitudeSpeedIn=0.0);
    int getAltitudeSpeed(double& altitudeSpeedOut, double& varAltitudeSpeedOut);

};


/////////////////////////////////////////
// Class RotationAnglesSensor
//
//   Description
//
/////////////////////////////////////////
class RotationAnglesSensor
{
protected:
    double yaw;
    double pitch;
    double roll;


    //White noise
protected:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;


public:
    RotationAnglesSensor();
    ~RotationAnglesSensor();

public:
    int setRotationAngles(double yawIn, double pitchIn, double rollIn);
    int getRotationAngles(double& yawOut, double& pitchOut, double& rollOut);

};


/////////////////////////////////////////
// Class GroundSpeedSensor
//
//   Description
//
/////////////////////////////////////////
class GroundSpeedSensor
{
protected:
    double vx;
    double vy;

    //White noise
protected:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

public:
    GroundSpeedSensor();
    ~GroundSpeedSensor();

public:
    int setGroundSpeed(double vxIn, double vyIn);
    int getGroundSpeed(double& vxOut, double& vyOut);

};


/////////////////////////////////////////
// Class PressureSensor
//
//   Description
//
/////////////////////////////////////////
class PressureSensor
{
protected:
    double fluidPressure;
    double varFluidPressure;


public:
    PressureSensor();
    ~PressureSensor();

public:
    int setPressure(double fluidPressureIn, double varFluidPressureIn);
    int getPressure(double& fluidPressureOut, double& varFluidPressureOut);

};


/////////////////////////////////////////
// Class Autopilot
//
//   Description
//
/////////////////////////////////////////
class Autopilot
{
    //Commands
protected:
    double pitchCommand;
    double rollCommand;
    double dYawCommand;
    double dAltitudeCommand;

public:
    Autopilot();
    ~Autopilot();


public:
    //Pitch & roll
    int setPitchRollCommand(double pitchCommandIn, double rollCommandIn);
    int getPitchRollCommand(double& pitchCommandOut, double& rollCommandOut);
    //dYaw
    int setDYawCommand(double dYawCommandIn);
    double getDYawCommand();
    //dAltitude
    int setDAltitudeCommand(double dAltitudeCommandIn);
    double getDAltitudeCommand();



};


/////////////////////////////////////////
// Class HLAutopilot
//
//   Description
//
/////////////////////////////////////////
//class HLAutopilot
//{
//protected:
//    DroneStateCommand::StateCommand HLCommand;


//public:
//    HLAutopilot();
//    ~HLAutopilot();


//public:
//    int setHLCommand(DroneStateCommand::StateCommand HLCommandIn);
//    DroneStateCommand::StateCommand getHLCommand();


//};




#endif
