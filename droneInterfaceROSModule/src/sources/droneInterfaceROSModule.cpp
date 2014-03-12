
#include "droneInterfaceROSModule.h"


////// IMU ////////
DroneInterfaceROSModule::DroneInterfaceROSModule() : DroneModule(droneModule::active, FREQ_INTERFACE)
{
    init();
    return;
}

DroneInterfaceROSModule::~DroneInterfaceROSModule()
{

    return;
}

void DroneInterfaceROSModule::init()
{
    //Imu

    //Battery
    BatteryMsgs.batteryPercent=0.0;

    //Altitude
    AltitudeMsgs.altitude=0.0;
    AltitudeMsgs.altitude_speed=0.0;
    AltitudeMsgs.var_altitude=0.0;
    AltitudeMsgs.var_altitude_speed=0.0;



    newBottomImage=false;
    newFrontImage=false;





    clearCmd();


    return;
}

void DroneInterfaceROSModule::close()
{

}

void DroneInterfaceROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration



    //Subscriber
    ImuSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_IMU, 1, &DroneInterfaceROSModule::imuCallback, this);
    TemperatureSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_TEMPERATURE, 1, &DroneInterfaceROSModule::temperatureCallback, this);
    MagnetometerSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_MAGNETOMETER, 1, &DroneInterfaceROSModule::magnetometerCallback, this);
    BatterySubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_BATTERY, 1, &DroneInterfaceROSModule::batteryCallback, this);
    AltitudeSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_ALTITUDE, 1, &DroneInterfaceROSModule::altitudeCallback, this);
    RotationAnglesSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_ROTATION_ANGLES, 1, &DroneInterfaceROSModule::rotationAnglesCallback, this);
    GroundSpeedSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_GROUND_SPEED, 1, &DroneInterfaceROSModule::groundSpeedCallback, this);
    PressureSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_PRESSURE, 1, &DroneInterfaceROSModule::pressureCallback, this);
    DroneStatusSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_STATUS, 1, &DroneInterfaceROSModule::droneStatusCallback, this);

    BottomCameraSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_BOTTOM_CAMERA, 1, &DroneInterfaceROSModule::bottomCameraCallback, this);
    FrontCameraSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_SENSOR_FRONT_CAMERA, 1, &DroneInterfaceROSModule::frontCameraCallback, this);


    //Cmd Subscribers
    DronePitchRollCmdSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_PITCH_ROLL_SUBSCRIPTION, 1, &DroneInterfaceROSModule::dronePitchRollCmdCallback, this);
    DroneDAltitudeCmdSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_DALTITUDE_SUBSCRIPTION, 1, &DroneInterfaceROSModule::droneDAltitudeCmdCallback, this);
    DroneDYawCmdSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_DYAW_SUBSCRIPTION, 1, &DroneInterfaceROSModule::droneDYawCmdCallback, this);
    DroneCommandSubs=n.subscribe(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_HL_COMMAND_SUBSCRIPTION, 1, &DroneInterfaceROSModule::droneCommandCallback, this);


    //Publishers
    DronePitchRollCmdPubl=n.advertise<droneMsgsROS::dronePitchRollCmd>(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_PITCH_ROLL_PUBLICATION,1, true);
    DroneDAltitudeCmdPubl=n.advertise<droneMsgsROS::droneDAltitudeCmd>(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_DALTITUDE_PUBLICATION,1, true);
    DroneDYawCmdPubl=n.advertise<droneMsgsROS::droneDYawCmd>(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_DYAW_PUBLICATION,1, true);
    DroneCommandPubl=n.advertise<droneMsgsROS::droneCommand>(DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_HL_COMMAND_PUBLICATION,1, true);


    //Flag of module opened
    droneModuleOpened=true;


    //End
    return;
}

//Reset
bool DroneInterfaceROSModule::resetValues()
{
    return true;
}

//Start
bool DroneInterfaceROSModule::startVal()
{
    return true;
}

//Stop
bool DroneInterfaceROSModule::stopVal()
{
    return true;
}

//Run
bool DroneInterfaceROSModule::run()
{
    return true;
}

void DroneInterfaceROSModule::clearCmd()
{
    //DronePitchRollCmd
    DronePitchRollCmdMsgs.pitchCmd=0.0;
    DronePitchRollCmdMsgs.rollCmd=0.0;

    DroneDAltitudeCmdMsgs.dAltitudeCmd=0.0;

    DroneDYawCmdMsgs.dYawCmd=0.0;
    return;
}


void DroneInterfaceROSModule::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ImuMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::temperatureCallback(const sensor_msgs::Temperature::ConstPtr& msg)
{
    TemperatureMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::magnetometerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    MagnetometerMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::batteryCallback(const droneMsgsROS::battery::ConstPtr& msg)
{
    BatteryMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::altitudeCallback(const droneMsgsROS::droneAltitude::ConstPtr& msg)
{
    AltitudeMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::rotationAnglesCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    RotationAnglesMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::groundSpeedCallback(const droneMsgsROS::vector2Stamped::ConstPtr& msg)
{
    GroundSpeedMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    PressureMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::droneStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg)
{
    DroneStatusMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    try
    {
        cvBottomImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        newBottomImage=true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        newBottomImage=false;
        return;
    }

    bottomImage=cvBottomImage->image;

    return;
}

void DroneInterfaceROSModule::frontCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    try
    {
        cvFrontImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        newFrontImage=true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        newFrontImage=false;
        return;
    }

    frontImage=cvFrontImage->image;

    return;
}



void DroneInterfaceROSModule::dronePitchRollCmdCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg)
{
    DronePitchRollCmdMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::droneDAltitudeCmdCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg)
{
    DroneDAltitudeCmdMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::droneDYawCmdCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg)
{
    DroneDYawCmdMsgs=*msg;
    return;
}

void DroneInterfaceROSModule::droneCommandCallback(const droneMsgsROS::droneCommand::ConstPtr& msg)
{
    DroneCommandMsgs=*msg;
    return;
}



bool DroneInterfaceROSModule::publishPitchRollCmd()
{
    DronePitchRollCmdPubl.publish(DronePitchRollCmdMsgs);

    return true;
}

bool DroneInterfaceROSModule::publishDAltitudeCmd()
{
    DroneDAltitudeCmdPubl.publish(DroneDAltitudeCmdMsgs);

    return true;
}

bool DroneInterfaceROSModule::publishDYawCmd()
{
    DroneDYawCmdPubl.publish(DroneDYawCmdMsgs);

    return true;
}

bool DroneInterfaceROSModule::publishDroneCmd()
{
    DroneCommandPubl.publish(DroneCommandMsgs);

    return true;
}

void DroneInterfaceROSModule::drone_take_off() {
    clearCmd();
    DroneCommandMsgs.command = droneMsgsROS::droneCommand::TAKE_OFF;
    publishDroneCmd();
}

void DroneInterfaceROSModule::drone_force_take_off() {
    clearCmd();

    //Reset if needed
    if( DroneStatusMsgs.status == droneMsgsROS::droneStatus::UNKNOWN ) {
        DroneCommandMsgs.command = droneMsgsROS::droneCommand::RESET;
        publishDroneCmd();
        ros::spinOnce();
    }
    //Wait until reseted!
    while( DroneStatusMsgs.status == droneMsgsROS::droneStatus::UNKNOWN) {
        ros::spinOnce();
    }
    drone_take_off();
}

void DroneInterfaceROSModule::drone_land() {
    clearCmd();
    DroneCommandMsgs.command = droneMsgsROS::droneCommand::LAND;
    publishDroneCmd();
}

void DroneInterfaceROSModule::drone_hover() {
    clearCmd();
    DroneCommandMsgs.command = droneMsgsROS::droneCommand::HOVER;
    publishDroneCmd();
}

void DroneInterfaceROSModule::drone_emergency_stop() {
    clearCmd();
    DroneCommandMsgs.command = droneMsgsROS::droneCommand::RESET;
    publishDroneCmd();
}

void DroneInterfaceROSModule::drone_reset() {
    drone_emergency_stop();
}

void DroneInterfaceROSModule::drone_move() {
    DroneCommandMsgs.command = droneMsgsROS::droneCommand::MOVE;
    publishDroneCmd();
}

std::stringstream *DroneInterfaceROSModule::getOdometryStream()
{
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "+Odometry measures:" << std::endl
        << " Battery: " << BatteryMsgs.batteryPercent << "%" << std::endl
        << " Yaw:   " << RotationAnglesMsgs.vector.z << " deg" << std::endl
        << " Pitch: " << RotationAnglesMsgs.vector.y << " deg" << std::endl
        << " Roll:  " << RotationAnglesMsgs.vector.x << " deg" << std::endl
        << " z (-H): "  << AltitudeMsgs.altitude << " m" << std::endl
        << " vx: " << GroundSpeedMsgs.vector.x    << " m/s" << std::endl
        << " vy: " << GroundSpeedMsgs.vector.y    << " m/s" << std::endl;
//        Note: ardrone does always returns 0 as vz/altitude_speed value
//        << " vz: " << AltitudeMsgs.altitude_speed << " m/s" << std::endl;
//        << " ax: " << ImuMsgs.linear_acceleration.x << " m/s2" << std::endl
//        << " ay: " << ImuMsgs.linear_acceleration.y << " m/s2" << std::endl
//        << " az: " << ImuMsgs.linear_acceleration.z << " m/s2" << std::endl
//        << " MagX: " << MagnetometerMsgs.vector.x << " (TBA)" << std::endl
//        << " MagY: " << MagnetometerMsgs.vector.y << " (TBA)" << std::endl
//        << " MagZ: " << MagnetometerMsgs.vector.z << " (TBA)" << std::endl
//        << " Press: " << PressureMsgs.fluid_pressure << " (TBA)" << std::endl;

    return &interface_printout_stream;
}

std::stringstream *DroneInterfaceROSModule::getDroneCommandsStream()
{
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "+Commands:" << std::endl
        << " Pitch: " << DronePitchRollCmdMsgs.pitchCmd     << std::endl
        << " Roll:  " << DronePitchRollCmdMsgs.rollCmd      << std::endl
        << " dYaw:  " << DroneDYawCmdMsgs.dYawCmd           << std::endl
        << " dh:    " << DroneDAltitudeCmdMsgs.dAltitudeCmd << std::endl;

    return &interface_printout_stream;
}


