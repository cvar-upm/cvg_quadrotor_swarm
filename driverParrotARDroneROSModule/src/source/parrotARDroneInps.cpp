//Drone
#include "parrotARDroneInps.h"


using namespace std;


////// DroneCommand ////////
DroneCommandROSModule::DroneCommandROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

DroneCommandROSModule::~DroneCommandROSModule()
{

    return;
}

void DroneCommandROSModule::init()
{
    CommandOutMsgs.angular.x=0.0;
    CommandOutMsgs.angular.y=0.0;
    CommandOutMsgs.angular.z=0.0;

    CommandOutMsgs.linear.x=0.0;
    CommandOutMsgs.linear.y=0.0;
    CommandOutMsgs.linear.z=0.0;


    return;
}

void DroneCommandROSModule::close()
{

}

void DroneCommandROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    CommandOutPubl = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    TakeOffPub=n.advertise<std_msgs::Empty>("ardrone/takeoff",1, true);
    LandPub=n.advertise<std_msgs::Empty>("ardrone/land",1, true);
    ResetPub=n.advertise<std_msgs::Empty>("ardrone/reset",1, true);


    //Subscribers
    PitchRollSubs=n.subscribe(DRONE_DRIVER_COMMAND_DRONE_COMMAND_PITCH_ROLL, 1, &DroneCommandROSModule::pitchRollCallback, this);
    AltitudeSubs=n.subscribe(DRONE_DRIVER_COMMAND_DRONE_COMMAND_DALTITUDE, 1, &DroneCommandROSModule::dAltitudeCallback, this);
    YawSubs=n.subscribe(DRONE_DRIVER_COMMAND_DRONE_COMMAND_DYAW, 1, &DroneCommandROSModule::dYawCallback, this);

    CommandSubs=n.subscribe(DRONE_DRIVER_COMMAND_DRONE_HL_COMMAND, 3, &DroneCommandROSModule::commandCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;


    //End
    return;
}

//Reset
bool DroneCommandROSModule::resetValues()
{
    return true;
}

//Start
bool DroneCommandROSModule::startVal()
{
    return true;
}

//Stop
bool DroneCommandROSModule::stopVal()
{
    return true;
}

//Run
bool DroneCommandROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void DroneCommandROSModule::pitchRollCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //header
    //TODO_JL

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

    CommandOutMsgs.linear.x = -pitchCmd;
    CommandOutMsgs.linear.y =  rollCmd;


    publishCommandValue();
    return;
}


void DroneCommandROSModule::dAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //header
    //TODO_JL

    //dAltitude
    double dAltitudeCmd=msg->dAltitudeCmd;
    if(dAltitudeCmd>1.0)
        dAltitudeCmd=1.0;
    else if(dAltitudeCmd<-1.0)
        dAltitudeCmd=-1.0;

    CommandOutMsgs.linear.z=dAltitudeCmd;


    publishCommandValue();
    return;
}


void DroneCommandROSModule::dYawCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;


    //header
    //TODO_JL

    //dAltitude
    double dYawCmd=-msg->dYawCmd;
    if(dYawCmd>1.0)
        dYawCmd=1.0;
    else if(dYawCmd<-1.0)
        dYawCmd=-1.0;

    CommandOutMsgs.angular.z=dYawCmd;


    publishCommandValue();
    return;
}


void DroneCommandROSModule::commandCallback(const droneMsgsROS::droneCommand::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;


    switch(msg->command)
    {
    case droneMsgsROS::droneCommand::TAKE_OFF:
        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        //Take off
        publishTakeOff();
        break;
    case droneMsgsROS::droneCommand::MOVE:
        //Clear
        CommandOutMsgs.angular.x=1.0;
        CommandOutMsgs.angular.y=1.0;
        //Publish
        publishCommandValue();
        break;
    case droneMsgsROS::droneCommand::LAND:
        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        //Land
        publishLand();
        break;
    case droneMsgsROS::droneCommand::HOVER:
        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        break;
    case droneMsgsROS::droneCommand::RESET:
        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        //Reset
        publishReset();
        break;
    default:
        break;
    }

    return;
}


bool DroneCommandROSModule::publishCommandValue()
{
    if(droneModuleOpened==false)
        return false;

    CommandOutPubl.publish(CommandOutMsgs);
    return true;
}


//Take off
bool DroneCommandROSModule::publishTakeOff()
{
    if(droneModuleOpened==false)
        return false;

    TakeOffPub.publish(EmptyMsg);
    return true;
}

//Land
bool DroneCommandROSModule::publishLand()
{
    if(droneModuleOpened==false)
        return false;


    LandPub.publish(EmptyMsg);
    return true;
}

//Reset
bool DroneCommandROSModule::publishReset()
{
    if(droneModuleOpened==false)
        return false;

    ResetPub.publish(EmptyMsg);
    return true;
}

