#include "droneElements.h"


////////// Battery
Battery::Battery()
{
    percentaje=100.0;
    return;
}

Battery::~Battery()
{
    return;
}

double Battery::getPercentaje()
{
    return percentaje;
}

int Battery::setPercenaje(double percentajeIn)
{
    percentaje=percentajeIn;
    return 1;
}



//////// IMU
IMU::IMU()
{
    return;
}
IMU::~IMU()
{
    return;
}


/////// Termometer
Termometer::Termometer()
{
    temperature=0.0;
    varTemperature=0.0;

    return;
}
Termometer::~Termometer()
{
    return;
}
int Termometer::setTemperature(double temperatureIn, double varTemperatureIn)
{
    temperature=temperatureIn;
    varTemperature=varTemperatureIn;
    return 1;
}
int Termometer::getTemperature(double& temperatureOut, double& varTemperatureOut)
{
    temperatureOut=temperature;
    varTemperatureOut=varTemperature;
    return 1;
}



////// Magnetometer
Magnetometer::Magnetometer()
{
    magX=0.0;
    magY=0.0;
    magZ=0.0;

    return;
}
Magnetometer::~Magnetometer()
{
    return;
}
int Magnetometer::setMagnetometer(double magXIn, double magYIn, double magZIn)
{
    magX=magXIn;
    magY=magYIn;
    magZ=magZIn;
    return 1;
}
int Magnetometer::getMagnetometer(double& magXOut, double& magYOut, double& magZOut)
{
    magXOut=magX;
    magYOut=magY;
    magZOut=magZ;
    return 1;
}


///////AltitudeSensor
AltitudeSensor::AltitudeSensor() :
    distribution(NOISE_MEAN_AltitudeSensor,NOISE_STD_AltitudeSensor)
{
    altitude=0.0;
    varAltitude=0.0;
    altitudeSpeed=0.0;
    varAltitudeSpeed=0.0;

    return;
}
AltitudeSensor::~AltitudeSensor()
{
    return;
}
int AltitudeSensor::setAltitude(double altitudeIn, double varAltitudeIn)
{
    altitude=altitudeIn;
    varAltitude=varAltitudeIn;
    return 1;
}
int AltitudeSensor::getAltitude(double& altitudeOut, double& varAltitudeOut)
{
    altitudeOut=altitude+distribution(generator);
    varAltitudeOut=varAltitude+distribution(generator);
    return 1;
}
int AltitudeSensor::setAltitudeSpeed(double altitudeSpeedIn, double varAltitudeSpeedIn)
{
    altitudeSpeed=altitudeSpeedIn;
    varAltitudeSpeed=varAltitudeSpeedIn;
    return 1;
}
int AltitudeSensor::getAltitudeSpeed(double& altitudeSpeedOut, double& varAltitudeSpeedOut)
{
    altitudeSpeedOut=altitudeSpeed+distribution(generator);
    varAltitudeSpeedOut=varAltitudeSpeed+distribution(generator);
    return 1;
}


//////// RotationAnglesSensor
RotationAnglesSensor::RotationAnglesSensor() :
    distribution(NOISE_MEAN_RotationAnglesSensor,NOISE_STD_RotationAnglesSensor)
{
    yaw=0.0;
    pitch=0.0;
    roll=0.0;

    return;
}
RotationAnglesSensor::~RotationAnglesSensor()
{
    return;
}
int RotationAnglesSensor::setRotationAngles(double yawIn, double pitchIn, double rollIn)
{
    yaw=yawIn;
    pitch=pitchIn;
    roll=rollIn;
    return 1;
}
int RotationAnglesSensor::getRotationAngles(double& yawOut, double& pitchOut, double& rollOut)
{
    yawOut=yaw+distribution(generator);
    pitchOut=pitch+distribution(generator);
    rollOut=roll+distribution(generator);
    return 1;
}


//////// GroundSpeedSensor
GroundSpeedSensor::GroundSpeedSensor() :
    distribution(NOISE_MEAN_GroundSpeedSensor,NOISE_STD_GroundSpeedSensor)
{
    vx=0.0;
    vy=0.0;

    return;
}
GroundSpeedSensor::~GroundSpeedSensor()
{
    return;
}
int GroundSpeedSensor::setGroundSpeed(double vxIn, double vyIn)
{
    vx=vxIn;
    vy=vyIn;
    return 1;
}
int GroundSpeedSensor::getGroundSpeed(double& vxOut, double& vyOut)
{
    vxOut=vx+distribution(generator);
    vyOut=vy+distribution(generator);
    return 1;
}


//////// PressureSensor
PressureSensor::PressureSensor()
{
    fluidPressure=0.0;
    varFluidPressure=0.0;

    return;
}
PressureSensor::~PressureSensor()
{
    return;
}
int PressureSensor::setPressure(double fluidPressureIn, double varFluidPressureIn)
{
    fluidPressure=fluidPressureIn;
    varFluidPressure=varFluidPressureIn;
    return 1;
}
int PressureSensor::getPressure(double& fluidPressureOut, double& varFluidPressureOut)
{
    fluidPressureOut=fluidPressure;
    varFluidPressureOut=varFluidPressure;
    return 1;
}




//////// Autopilot
Autopilot::Autopilot()
{

}
Autopilot::~Autopilot()
{

}
//Pitch & roll
int Autopilot::setPitchRollCommand(double pitchCommandIn, double rollCommandIn)
{
    pitchCommand=pitchCommandIn;
    rollCommand=rollCommandIn;
    return 1;
}
int Autopilot::getPitchRollCommand(double& pitchCommandOut, double& rollCommandOut)
{
    pitchCommandOut=pitchCommand;
    rollCommandOut=rollCommand;
    return 1;
}
//dYaw
int Autopilot::setDYawCommand(double dYawCommandIn)
{
    dYawCommand=dYawCommandIn;
    return 1;
}
double Autopilot::getDYawCommand()
{
    return dYawCommand;
}
//dAltitude
int Autopilot::setDAltitudeCommand(double dAltitudeCommandIn)
{
    dAltitudeCommand=dAltitudeCommandIn;
    return 1;
}
double Autopilot::getDAltitudeCommand()
{
    return dAltitudeCommand;
}



//////// HLAutopilot
//HLAutopilot()
//{
//    return;
//}
//~HLAutopilot()
//{

//}
//int setHLCommand(DroneStateCommand::StateCommand HLCommandIn)
//{

//}
//DroneStateCommand::StateCommand getHLCommand()
//{

//}
