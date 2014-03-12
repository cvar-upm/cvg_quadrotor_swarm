//Drone
#include "droneSimulatorROSModule.h"


using namespace std;


////// DroneSimulatorROSModule ////////
DroneSimulatorROSModule::DroneSimulatorROSModule() : DroneModule(droneModule::active, DRONE_SIMULATOR_RATE) ,
  MyDroneSimulator(idDrone,stackPath)
{
    init();
    return;
}

DroneSimulatorROSModule::~DroneSimulatorROSModule()
{

    return;
}

void DroneSimulatorROSModule::init()
{

    return;
}

void DroneSimulatorROSModule::close()
{

}

void DroneSimulatorROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration



    //Subscribers
    PitchRollSubs=n.subscribe(DRONE_SIMULATOR_COMMAND_DRONE_COMMAND_PITCH_ROLL, 1, &DroneSimulatorROSModule::pitchRollCallback, this);
    AltitudeSubs=n.subscribe(DRONE_SIMULATOR_COMMAND_DRONE_COMMAND_DALTITUDE, 1, &DroneSimulatorROSModule::dAltitudeCallback, this);
    YawSubs=n.subscribe(DRONE_SIMULATOR_COMMAND_DRONE_COMMAND_DYAW, 1, &DroneSimulatorROSModule::dYawCallback, this);

    CommandSubs=n.subscribe(DRONE_SIMULATOR_COMMAND_DRONE_HL_COMMAND, 3, &DroneSimulatorROSModule::commandCallback, this);


    //Publishers
    ImuPubl = n.advertise<sensor_msgs::Imu>(DRONE_SIMULATOR_SENSOR_IMU, 1, true);
    TemperaturePubl = n.advertise<sensor_msgs::Temperature>(DRONE_SIMULATOR_SENSOR_TEMPERATURE, 1, true);
    MagnetometerPubl = n.advertise<geometry_msgs::Vector3Stamped>(DRONE_SIMULATOR_SENSOR_MAGNETOMETER, 1, true);
    BatteryPubl = n.advertise<droneMsgsROS::battery>(DRONE_SIMULATOR_SENSOR_BATTERY, 1, true);
    AltitudePubl = n.advertise<droneMsgsROS::droneAltitude>(DRONE_SIMULATOR_SENSOR_ALTITUDE, 1, true);
    RotationAnglesPubl = n.advertise<geometry_msgs::Vector3Stamped>(DRONE_SIMULATOR_SENSOR_ROTATION_ANGLES, 1, true);
    GroundSpeedPubl = n.advertise<droneMsgsROS::vector2Stamped>(DRONE_SIMULATOR_SENSOR_GROUND_SPEED, 1, true);
    PressurePubl = n.advertise<sensor_msgs::FluidPressure>(DRONE_SIMULATOR_SENSOR_PRESSURE, 1, true);
    DroneStatusPubl = n.advertise<droneMsgsROS::droneStatus>(DRONE_SIMULATOR_SENSOR_STATUS, 1, true);


    //Publisher
    InternalDronePosePubl = n.advertise<droneMsgsROS::dronePoseStamped>(DRONE_SIMULATOR_INTERNAL_POSE, 1, true);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;


    //End
    return;
}

//Reset
bool DroneSimulatorROSModule::resetValues()
{
    return true;
}

//Start
bool DroneSimulatorROSModule::startVal()
{
    return true;
}

//Stop
bool DroneSimulatorROSModule::stopVal()
{
    return true;
}

//Run
bool DroneSimulatorROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }


    //Evaluate DroneSimulator
    MyDroneSimulator.run();


    //Publish everything
    publishImuValue();
    publishTemperatureValue();
    publishMagnetometerValue();
    publishBatteryValue();
    publishAltitudeValue();
    publishRotationAnglesValue();
    publishGroundSpeedValue();
    publishPressureValue();
    publishDroneStatusValue();

    publishInternalDronePose();


    //End
    return true;
}


void DroneSimulatorROSModule::pitchRollCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg)
{
    //Pitch
    double pitchCmd=msg->pitchCmd;
    if(pitchCmd>1.0)
        pitchCmd=1.0;
    else if(pitchCmd<-1.0)
        pitchCmd=-1.0;

    //Roll
    double rollCmd=-msg->rollCmd;
    if(rollCmd>1.0)
        rollCmd=1.0;
    else if(rollCmd<-1.0)
        rollCmd=-1.0;

    MyDroneSimulator.DroneAutopilot.setPitchRollCommand(pitchCmd,rollCmd);


    return;
}


void DroneSimulatorROSModule::dAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg)
{
    //dAltitude
    double dAltitudeCmd=msg->dAltitudeCmd;
    if(dAltitudeCmd>1.0)
        dAltitudeCmd=1.0;
    else if(dAltitudeCmd<-1.0)
        dAltitudeCmd=-1.0;

    MyDroneSimulator.DroneAutopilot.setDAltitudeCommand(dAltitudeCmd);

    return;
}


void DroneSimulatorROSModule::dYawCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg)
{
    //dYawCallback
    double dYawCmd=-msg->dYawCmd;
    if(dYawCmd>1.0)
        dYawCmd=1.0;
    else if(dYawCmd<-1.0)
        dYawCmd=-1.0;

    MyDroneSimulator.DroneAutopilot.setDYawCommand(dYawCmd);


    return;
}


void DroneSimulatorROSModule::commandCallback(const droneMsgsROS::droneCommand::ConstPtr& msg)
{
    switch(msg->command)
    {
    case droneMsgsROS::droneCommand::TAKE_OFF:
        MyDroneSimulator.commandDrone(DroneStateCommand::StateCommand::TAKE_OFF);
        break;
    case droneMsgsROS::droneCommand::MOVE:
        MyDroneSimulator.commandDrone(DroneStateCommand::StateCommand::MOVE);
        break;
    case droneMsgsROS::droneCommand::LAND:
        MyDroneSimulator.commandDrone(DroneStateCommand::StateCommand::LAND);
        break;
    case droneMsgsROS::droneCommand::HOVER:
        MyDroneSimulator.commandDrone(DroneStateCommand::StateCommand::HOVER);
        break;
    case droneMsgsROS::droneCommand::RESET:
        MyDroneSimulator.commandDrone(DroneStateCommand::StateCommand::RESET);
        break;
    default:
        break;
    }

    return;
}


bool DroneSimulatorROSModule::publishImuValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    ImuMsgs.header.stamp=ros::Time::now();
    //TODO JL

    //Publish
    ImuPubl.publish(ImuMsgs);
    return true;
}


bool DroneSimulatorROSModule::publishTemperatureValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    TemperatureMsgs.header.stamp=ros::Time::now();
    MyDroneSimulator.DroneTermometer.getTemperature(TemperatureMsgs.temperature,TemperatureMsgs.variance);

    //Publish
    TemperaturePubl.publish(TemperatureMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishMagnetometerValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    MagnetometerMsgs.header.stamp=ros::Time::now();
    MyDroneSimulator.DroneMagnetometer.getMagnetometer(MagnetometerMsgs.vector.x,MagnetometerMsgs.vector.y,MagnetometerMsgs.vector.z);

    //Publish
    MagnetometerPubl.publish(MagnetometerMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishBatteryValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    BatteryMsgs.header.stamp=ros::Time::now();
    BatteryMsgs.batteryPercent=MyDroneSimulator.DroneBattery.getPercentaje();

    //Publish
    BatteryPubl.publish(BatteryMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishAltitudeValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    AltitudeMsgs.header.stamp=ros::Time::now();
    MyDroneSimulator.DroneAltitudeSensor.getAltitude(AltitudeMsgs.altitude,AltitudeMsgs.var_altitude);
    MyDroneSimulator.DroneAltitudeSensor.getAltitudeSpeed(AltitudeMsgs.altitude_speed,AltitudeMsgs.var_altitude_speed);

    //Publish
    AltitudePubl.publish(AltitudeMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishRotationAnglesValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    RotationAnglesMsgs.header.stamp=ros::Time::now();
    MyDroneSimulator.DroneRotationAnglesSensor.getRotationAngles(RotationAnglesMsgs.vector.z,RotationAnglesMsgs.vector.y,RotationAnglesMsgs.vector.x);

    //Publish
    RotationAnglesPubl.publish(RotationAnglesMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishGroundSpeedValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    GroundSpeedMsgs.header.stamp=ros::Time::now();
    MyDroneSimulator.DroneGroundSpeedSensor.getGroundSpeed(GroundSpeedMsgs.vector.x,GroundSpeedMsgs.vector.y);

    //Publish
    GroundSpeedPubl.publish(GroundSpeedMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishPressureValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    PressureMsgs.header.stamp=ros::Time::now();
    MyDroneSimulator.DronePressureSensor.getPressure(PressureMsgs.fluid_pressure,PressureMsgs.variance);

    //Publish
    PressurePubl.publish(PressureMsgs);
    return true;
}

bool DroneSimulatorROSModule::publishDroneStatusValue()
{
    if(droneModuleOpened==false)
        return false;

    //Fill the msg
    DroneStatusMsgs.header.stamp=ros::Time::now();
    switch(MyDroneSimulator.getCurrentDroneState())
    {
    case DroneState::ModeType::UNKNOWN:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::UNKNOWN;
        break;
    case DroneState::ModeType::INIT:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::INITED;
        break;
    case DroneState::ModeType::LANDED:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::LANDED;
        break;
    case DroneState::ModeType::FLYING:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::FLYING;
        break;
    case DroneState::ModeType::HOVERING:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::HOVERING;
        break;
//    case DroneState::ModeType::TEST:
//        DroneStatusMsgs.status=droneMsgsROS::droneStatus::UNKNOWN;
//        break;
    case DroneState::ModeType::TAKING_OFF:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::TAKING_OFF;
        break;
//    case DroneState::ModeType::GOTO_FIX_POINT:
//        DroneStatusMsgs.status=droneMsgsROS::droneStatus::FLYING;
//        break;
    case DroneState::ModeType::LANDING:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::LANDING;
        break;
    case DroneState::ModeType::LOOPING:
        DroneStatusMsgs.status=droneMsgsROS::droneStatus::LOOPING;
        break;

    }

    //Publish
    DroneStatusPubl.publish(DroneStatusMsgs);
    return true;
}


bool DroneSimulatorROSModule::publishInternalDronePose()
{
    if(droneModuleOpened==false)
        return false;

    //Fill msgs
    InternalDronePoseMsgs.header.stamp=ros::Time::now();
    double x, y, z, yaw, pitch, roll;
    MyDroneSimulator.getPosition_drone_GMR_wrt_GFF(x, y, z, yaw, pitch, roll);
    InternalDronePoseMsgs.pose.x=static_cast<float>(x);
    InternalDronePoseMsgs.pose.y=static_cast<float>(y);
    InternalDronePoseMsgs.pose.z=static_cast<float>(z);
    InternalDronePoseMsgs.pose.yaw=static_cast<float>(yaw);
    InternalDronePoseMsgs.pose.pitch=static_cast<float>(pitch);
    InternalDronePoseMsgs.pose.roll=static_cast<float>(roll);
    InternalDronePoseMsgs.pose.reference_frame="GFF";
    InternalDronePoseMsgs.pose.target_frame="drone_GMR";
    InternalDronePoseMsgs.pose.YPR_system="wYvPuR";

    //Publish
    InternalDronePosePubl.publish(InternalDronePoseMsgs);
    return true;
}
