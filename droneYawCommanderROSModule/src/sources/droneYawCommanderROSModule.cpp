/*
 * 
 *
 *  Created on: 
 *      Author: Jose Luis Sanchez-Lopez
 */


#include "droneYawCommanderROSModule.h"



using namespace std;




DroneYawCommander::DroneYawCommander() : DroneModule(droneModule::active,FREQ_YAW_PLANNER)
{
    if(!init())
        cout<<"Error init"<<endl;


    return;
}



DroneYawCommander::~DroneYawCommander()
{
	close();
	return;
}

void DroneYawCommander::open(ros::NodeHandle & nIn, std::string moduleName)
{
	//Node
    DroneModule::open(nIn,moduleName);


	
    //// Topics ///
    ///subs
    // rostopic pub -1 /drone1/ArucoSlam_EstimatedPose droneMsgsROS/dronePose -- 123 0.0 0.0 1.5 0.0 0.0 0.0 "a" "a" "a"
    dronePoseSubs = n.subscribe(DRONE_YAW_PLANNER_POSE_SUBS, 1, &DroneYawCommander::dronePoseCallback, this);

    dronePointToLookSub = n.subscribe(DRONE_YAW_PLANNER_POINT_TO_LOOK_SUB, 1, &DroneYawCommander::dronePointToLookCallback, this);
    // rostopic pub -1 /drone1/droneYawToLook droneMsgsROS/droneYawRefCommand -- 123 1.3
    droneYawToLookSub = n.subscribe(DRONE_YAW_PLANNER_YAW_TO_LOOK_SUB, 1, &DroneYawCommander::droneYawToLookCallback, this);


    ///pub
    droneYawRefCommandPub=n.advertise<droneMsgsROS::droneYawRefCommand>(DRONE_YAW_PLANNER_YAW_REF_CMD_PUB, 1, true);





    //Flag of module opened
    droneModuleOpened=true;



	//End
	return;
}

void DroneYawCommander::dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg)
{
    dronePoseMsg.time=msg->time;
    dronePoseMsg.x=msg->x;
    dronePoseMsg.y=msg->y;
    dronePoseMsg.z=msg->z;
    dronePoseMsg.yaw=msg->yaw;
    dronePoseMsg.pitch=msg->pitch;
    dronePoseMsg.roll=msg->roll;

    return;
}

void DroneYawCommander::dronePointToLookCallback(const droneMsgsROS::dronePositionRefCommand::ConstPtr& msg)
{
    dronePointToLookMsg.x=msg->x;
    dronePointToLookMsg.y=msg->y;
    dronePointToLookMsg.z=msg->z;

    clearFlags();
    flagPointToLook=true;

    return;
}


void DroneYawCommander::droneYawToLookCallback(const droneMsgsROS::droneYawRefCommand::ConstPtr& msg)
{
    droneYawToLookMsg.header=msg->header;
    droneYawToLookMsg.yaw=msg->yaw;

    clearFlags();
    flagYawToLook=true;

    return;
}



int DroneYawCommander::publishYawRefCommand(droneMsgsROS::droneYawRefCommand droneYawRefCommandIn)
{
    if(droneModuleOpened==false)
        return 0;

    droneYawRefCommandPub.publish(droneYawRefCommandIn);

    return 1;
}




bool DroneYawCommander::init()
{
    dronePoseMsg.time=0.0;
    dronePoseMsg.x=0.0;
    dronePoseMsg.y=0.0;
    dronePoseMsg.z=0.0;
    dronePoseMsg.yaw=0.0;
    dronePoseMsg.pitch=0.0;
    dronePoseMsg.roll=0.0;

    droneYawRefCommandMsg.yaw=0.0;

    clearFlags();

    //end
    return true;
}



void DroneYawCommander::close()
{

    DroneModule::close();
	return;
}



bool DroneYawCommander::resetValues()
{
    droneYawRefCommandMsg.yaw=0.0;

    clearFlags();

    return true;

}



bool DroneYawCommander::startVal()
{


    //End
    return DroneModule::startVal();
}



bool DroneYawCommander::stopVal()
{
    return DroneModule::stopVal();
}



bool DroneYawCommander::run()
{
    if(!DroneModule::run())
        return false;

    if(droneModuleOpened==false)
        return false;



    if(flagYawToLook || flagPointToLook)
    {
        //Yaw defined by Yaw
        if(flagYawToLook)
        {
            droneYawRefCommandMsg.yaw=droneYawToLookMsg.yaw;
        }

        //Yaw defined by point
        if(flagPointToLook)
        {
            droneYawRefCommandMsg.header.stamp = ros::Time::now();
            droneYawRefCommandMsg.yaw = atan2( dronePointToLookMsg.y - dronePoseMsg.y,
                                               dronePointToLookMsg.x - dronePoseMsg.x);
        }

        //publish
        publishYawRefCommand(droneYawRefCommandMsg);
    }



    return false;
}


int DroneYawCommander::clearFlags()
{
    flagYawToLook=false;
    flagPointToLook=false;

    return 1;
}

