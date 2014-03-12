//Drone
#include "parrotARDroneOuts.h"


using namespace std;


////// IMU ////////
ImuROSModule::ImuROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

ImuROSModule::~ImuROSModule()
{

    return;
}

void ImuROSModule::init()
{

}

void ImuROSModule::close()
{

}

void ImuROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    ImuPubl = n.advertise<sensor_msgs::Imu>(DRONE_DRIVER_SENSOR_IMU, 1, true);


    //Subscriber
    ImuSubs=n.subscribe("ardrone/imu", 1, &ImuROSModule::imuCallback, this);


    //Flag of module opened
    droneModuleOpened=true;


    //Auto-Start module
    moduleStarted=true;


    //End
    return;
}

//Reset
bool ImuROSModule::resetValues()
{
    return true;
}

//Start
bool ImuROSModule::startVal()
{
    return true;
}

//Stop
bool ImuROSModule::stopVal()
{
    return true;
}

//Run
bool ImuROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void ImuROSModule::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;


    ImuMsgs=*msg;

    // Linear acceleration [m/s2],   mavwork reference frame
    ImuMsgs.linear_acceleration.y*=-1.0;
    ImuMsgs.linear_acceleration.z*=-1.0;

    // Angular velocity [rad/s],   mavwork reference frame
    ImuMsgs.angular_velocity.y*=-1.0;
    ImuMsgs.angular_velocity.z*=-1.0;

    // Orientation quaterniun
    //TODO_JL


    publishImuValue();
    return;
}


bool ImuROSModule::publishImuValue()
{
    if(droneModuleOpened==false)
        return false;

    ImuPubl.publish(ImuMsgs);
    return true;
}





////// Temperature ////////
TemperatureROSModule::TemperatureROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

TemperatureROSModule::~TemperatureROSModule()
{

    return;
}

void TemperatureROSModule::init()
{

}

void TemperatureROSModule::close()
{

}

void TemperatureROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    TemperaturePubl = n.advertise<sensor_msgs::Temperature>(DRONE_DRIVER_SENSOR_TEMPERATURE, 1, true);


    //Subscriber
    TemperatureSubs=n.subscribe("ardrone/navdata", 1, &TemperatureROSModule::temperatureCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool TemperatureROSModule::resetValues()
{
    return true;
}

//Start
bool TemperatureROSModule::startVal()
{
    return true;
}

//Stop
bool TemperatureROSModule::stopVal()
{
    return true;
}

//Run
bool TemperatureROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void TemperatureROSModule::temperatureCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read temperature from navdata
    TemperatureMsgs.header=msg->header;
    TemperatureMsgs.temperature=msg->temp; //Temperature needs to be put in degrees!!
    TemperatureMsgs.variance=0.0;

    publishTemperatureValue();
    return;
}


bool TemperatureROSModule::publishTemperatureValue()
{
    if(droneModuleOpened==false)
        return false;

    TemperaturePubl.publish(TemperatureMsgs);
    return true;
}




////// Magnetometer ////////
MagnetometerROSModule::MagnetometerROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

MagnetometerROSModule::~MagnetometerROSModule()
{

    return;
}

void MagnetometerROSModule::init()
{

}

void MagnetometerROSModule::close()
{

}

void MagnetometerROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    MagnetometerPubl = n.advertise<geometry_msgs::Vector3Stamped>(DRONE_DRIVER_SENSOR_MAGNETOMETER, 1, true);


    //Subscriber
    MagnetometerSubs=n.subscribe("ardrone/mag", 1, &MagnetometerROSModule::magnetometerCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;


    //End
    return;
}

//Reset
bool MagnetometerROSModule::resetValues()
{
    return true;
}

//Start
bool MagnetometerROSModule::startVal()
{
    return true;
}

//Stop
bool MagnetometerROSModule::stopVal()
{
    return true;
}

//Run
bool MagnetometerROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void MagnetometerROSModule::magnetometerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    MagnetometerMsgs=*msg;

    // [TBA?],mavwork reference frame
    MagnetometerMsgs.vector.y*=-1.0;
    MagnetometerMsgs.vector.z*=-1.0;

    publishMagnetometerValue();
    return;
}


bool MagnetometerROSModule::publishMagnetometerValue()
{
    if(droneModuleOpened==false)
        return false;

    MagnetometerPubl.publish(MagnetometerMsgs);
    return true;
}





////// Battery ////////
BatteryROSModule::BatteryROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

BatteryROSModule::~BatteryROSModule()
{

    return;
}

void BatteryROSModule::init()
{

}

void BatteryROSModule::close()
{

}

void BatteryROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    BatteryPubl = n.advertise<droneMsgsROS::battery>(DRONE_DRIVER_SENSOR_BATTERY, 1, true);


    //Subscriber
    BatterySubs=n.subscribe("ardrone/navdata", 1, &BatteryROSModule::batteryCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool BatteryROSModule::resetValues()
{
    return true;
}

//Start
bool BatteryROSModule::startVal()
{
    return true;
}

//Stop
bool BatteryROSModule::stopVal()
{
    return true;
}

//Run
bool BatteryROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void BatteryROSModule::batteryCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read Battery from navdata
    BatteryMsgs.header=msg->header;
    BatteryMsgs.batteryPercent=msg->batteryPercent;

    publishBatteryValue();
    return;
}


bool BatteryROSModule::publishBatteryValue()
{
    if(droneModuleOpened==false)
        return false;

    BatteryPubl.publish(BatteryMsgs);
    return true;
}




////// Altitude ////////
AltitudeROSModule::AltitudeROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

AltitudeROSModule::~AltitudeROSModule()
{

    return;
}

void AltitudeROSModule::init()
{

}

void AltitudeROSModule::close()
{

}

void AltitudeROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    AltitudePubl = n.advertise<droneMsgsROS::droneAltitude>(DRONE_DRIVER_SENSOR_ALTITUDE, 1, true);


    //Subscriber
    AltitudeSubs=n.subscribe("ardrone/navdata", 1, &AltitudeROSModule::altitudeCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool AltitudeROSModule::resetValues()
{
    return true;
}

//Start
bool AltitudeROSModule::startVal()
{
    return true;
}

//Stop
bool AltitudeROSModule::stopVal()
{
    return true;
}

//Run
bool AltitudeROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void AltitudeROSModule::altitudeCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read Altitude from navdata
    AltitudeMsgs.header=msg->header;
    //Altitude needs to be put in [m], mavwork reference frame!!
    AltitudeMsgs.altitude=-msg->altd/1000.0;
    AltitudeMsgs.var_altitude=0.0;
    // [m/s], mavwork reference frame
    AltitudeMsgs.altitude_speed=-msg->vz/1000.0;
    AltitudeMsgs.var_altitude_speed=0.0;

    publishAltitudeValue();
    return;
}


bool AltitudeROSModule::publishAltitudeValue()
{
    if(droneModuleOpened==false)
        return false;

    AltitudePubl.publish(AltitudeMsgs);
    return true;
}



////// RotationAngles ////////
RotationAnglesROSModule::RotationAnglesROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

RotationAnglesROSModule::~RotationAnglesROSModule()
{

    return;
}

void RotationAnglesROSModule::init()
{

}

void RotationAnglesROSModule::close()
{

}

void RotationAnglesROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    RotationAnglesPubl = n.advertise<geometry_msgs::Vector3Stamped>(DRONE_DRIVER_SENSOR_ROTATION_ANGLES, 1, true);


    //Subscriber
    RotationAnglesSubs=n.subscribe("ardrone/navdata", 1, &RotationAnglesROSModule::rotationAnglesCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool RotationAnglesROSModule::resetValues()
{
    return true;
}

//Start
bool RotationAnglesROSModule::startVal()
{
    return true;
}

//Stop
bool RotationAnglesROSModule::stopVal()
{
    return true;
}

//Run
bool RotationAnglesROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void RotationAnglesROSModule::rotationAnglesCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    RotationAnglesMsgs.header=msg->header;
    // rad,   mavwork reference frame
    RotationAnglesMsgs.vector.x=msg->rotX;//*M_PI/180.0;
    RotationAnglesMsgs.vector.y=-msg->rotY;//*M_PI/180.0;
    RotationAnglesMsgs.vector.z=-msg->rotZ;//*M_PI/180.0;

    publishRotationAnglesValue();
    return;
}


bool RotationAnglesROSModule::publishRotationAnglesValue()
{
    if(droneModuleOpened==false)
        return false;

    RotationAnglesPubl.publish(RotationAnglesMsgs);
    return true;
}





////// GroundSpeed ////////
GroundSpeedROSModule::GroundSpeedROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

GroundSpeedROSModule::~GroundSpeedROSModule()
{

    return;
}

void GroundSpeedROSModule::init()
{

}

void GroundSpeedROSModule::close()
{

}

void GroundSpeedROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    GroundSpeedPubl = n.advertise<droneMsgsROS::vector2Stamped>(DRONE_DRIVER_SENSOR_GROUND_SPEED, 1, true);


    //Subscriber
    GroundSpeedSubs=n.subscribe("ardrone/navdata", 1, &GroundSpeedROSModule::groundSpeedCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool GroundSpeedROSModule::resetValues()
{
    return true;
}

//Start
bool GroundSpeedROSModule::startVal()
{
    return true;
}

//Stop
bool GroundSpeedROSModule::stopVal()
{
    return true;
}

//Run
bool GroundSpeedROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void GroundSpeedROSModule::groundSpeedCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read GroundSpeed from navdata
    GroundSpeedMsgs.header=msg->header;

    // [m/s], mavwork reference frame
    GroundSpeedMsgs.vector.x=msg->vx/1000;
    GroundSpeedMsgs.vector.y=-msg->vy/1000;

    publishGroundSpeedValue();
    return;
}


bool GroundSpeedROSModule::publishGroundSpeedValue()
{
    if(droneModuleOpened==false)
        return false;

    GroundSpeedPubl.publish(GroundSpeedMsgs);
    return true;
}


////// Pressure ////////
PressureROSModule::PressureROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

PressureROSModule::~PressureROSModule()
{

    return;
}

void PressureROSModule::init()
{

}

void PressureROSModule::close()
{

}

void PressureROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    PressurePubl = n.advertise<sensor_msgs::FluidPressure>(DRONE_DRIVER_SENSOR_PRESSURE, 1, true);


    //Subscriber
    PressureSubs=n.subscribe("ardrone/navdata", 1, &PressureROSModule::pressureCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool PressureROSModule::resetValues()
{
    return true;
}

//Start
bool PressureROSModule::startVal()
{
    return true;
}

//Stop
bool PressureROSModule::stopVal()
{
    return true;
}

//Run
bool PressureROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void PressureROSModule::pressureCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read Pressure from navdata
    PressureMsgs.header=msg->header;
    PressureMsgs.fluid_pressure=msg->pressure; //Pressure needs to be put in degrees!!
    PressureMsgs.variance=0.0;

    publishPressureValue();
    return;
}


bool PressureROSModule::publishPressureValue()
{
    if(droneModuleOpened==false)
        return false;

    PressurePubl.publish(PressureMsgs);
    return true;
}



////// DroneStatus ////////
DroneStatusROSModule::DroneStatusROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

DroneStatusROSModule::~DroneStatusROSModule()
{

    return;
}

void DroneStatusROSModule::init()
{

}

void DroneStatusROSModule::close()
{

}

void DroneStatusROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    DroneStatusPubl = n.advertise<droneMsgsROS::droneStatus>(DRONE_DRIVER_SENSOR_STATUS, 1, true);


    //Subscriber
    DroneStatusSubs=n.subscribe("ardrone/navdata", 1, &DroneStatusROSModule::droneStatusCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool DroneStatusROSModule::resetValues()
{
    return true;
}

//Start
bool DroneStatusROSModule::startVal()
{
    return true;
}

//Stop
bool DroneStatusROSModule::stopVal()
{
    return true;
}

//Run
bool DroneStatusROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void DroneStatusROSModule::droneStatusCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read DroneStatus from navdata
    DroneStatusMsgs.header=msg->header;

    switch(msg->state)
    {
    case 0:
    default:
        DroneStatusMsgs.status=DroneStatusMsgs.UNKNOWN;
        break;
    case 1:
        DroneStatusMsgs.status=DroneStatusMsgs.INITED;
        break;
    case 2:
        DroneStatusMsgs.status=DroneStatusMsgs.LANDED;
        break;
    case 3:
        DroneStatusMsgs.status=DroneStatusMsgs.FLYING;
        break;
    case 4:
        DroneStatusMsgs.status=DroneStatusMsgs.HOVERING;
        break;
    case 5:
        DroneStatusMsgs.status=DroneStatusMsgs.UNKNOWN;
        break;
    case 6:
        DroneStatusMsgs.status=DroneStatusMsgs.TAKING_OFF;
        break;
    case 7:
        DroneStatusMsgs.status=DroneStatusMsgs.FLYING;
        break;
    case 8:
        DroneStatusMsgs.status=DroneStatusMsgs.LANDING;
        break;
    case 9:
        DroneStatusMsgs.status=DroneStatusMsgs.LOOPING;
        break;
    }


    publishDroneStatusValue();
    return;
}


bool DroneStatusROSModule::publishDroneStatusValue()
{
    if(droneModuleOpened==false)
        return false;

    DroneStatusPubl.publish(DroneStatusMsgs);
    return true;
}



////// BottomCamera ////////
BottomCameraROSModule::BottomCameraROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

BottomCameraROSModule::~BottomCameraROSModule()
{

    return;
}

void BottomCameraROSModule::init()
{

}

void BottomCameraROSModule::close()
{

}

void BottomCameraROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    image_transport::ImageTransport it(n);


    //Publisher
    BottomCameraPubl = it.advertiseCamera(DRONE_DRIVER_SENSOR_BOTTOM_CAMERA, 1, true);


    //Subscriber
    BottomCameraSubs=it.subscribeCamera("ardrone/bottom/image_raw", 1, &BottomCameraROSModule::bottomCameraCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool BottomCameraROSModule::resetValues()
{
    return true;
}

//Start
bool BottomCameraROSModule::startVal()
{
    return true;
}

//Stop
bool BottomCameraROSModule::stopVal()
{
    return true;
}

//Run
bool BottomCameraROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void BottomCameraROSModule::bottomCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read BottomCamera
    BottomCameraMsgs=image_msg;
    BottomCameraInfoMsgs=info_msg;

    publishBottomCameraValue();
    return;
}


bool BottomCameraROSModule::publishBottomCameraValue()
{
    if(droneModuleOpened==false)
        return false;

    BottomCameraPubl.publish(BottomCameraMsgs,BottomCameraInfoMsgs);
    return true;
}



////// FrontCamera ////////
FrontCameraROSModule::FrontCameraROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

FrontCameraROSModule::~FrontCameraROSModule()
{

    return;
}

void FrontCameraROSModule::init()
{

}

void FrontCameraROSModule::close()
{

}

void FrontCameraROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    image_transport::ImageTransport it(n);


    //Publisher
    FrontCameraPubl = it.advertiseCamera(DRONE_DRIVER_SENSOR_FRONT_CAMERA, 1, true);


    //Subscriber
    FrontCameraSubs=it.subscribeCamera("ardrone/front/image_raw", 1, &FrontCameraROSModule::frontCameraCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Reset
bool FrontCameraROSModule::resetValues()
{
    return true;
}

//Start
bool FrontCameraROSModule::startVal()
{
    return true;
}

//Stop
bool FrontCameraROSModule::stopVal()
{
    return true;
}

//Run
bool FrontCameraROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void FrontCameraROSModule::frontCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read FrontCamera
    FrontCameraMsgs=image_msg;
    FrontCameraInfoMsgs=info_msg;

    publishFrontCameraValue();
    return;
}


bool FrontCameraROSModule::publishFrontCameraValue()
{
    if(droneModuleOpened==false)
        return false;

    FrontCameraPubl.publish(FrontCameraMsgs,FrontCameraInfoMsgs);
    return true;
}
