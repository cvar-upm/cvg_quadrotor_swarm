/*
 * 
 *
 *  Created on: 
 *      Author: Jose Luis Sanchez-Lopez
 */

#ifndef DRONE_YAW_PLANNER_H
#define DRONE_YAW_PLANNER_H



////// ROS  ///////
#include "ros/ros.h"



//Math
#include <math.h>


#include <cstdlib>
#include <ctime>



#include <iostream>




//Drone module
#include "droneModuleROS.h"



////Msgs
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneYawRefCommand.h"
#include "droneMsgsROS/dronePositionRefCommand.h"


//Communications
#include "communication_definition.h"


//Freq
#define FREQ_YAW_PLANNER    20.0






class DroneYawCommander : public DroneModule
{	
    //Flags
private:
    bool flagYawToLook;
    bool flagPointToLook;

    int clearFlags();



/////Communications!!

    //drone pose (subs)
private:
    droneMsgsROS::dronePose dronePoseMsg;
    /////Subscribers
    ros::Subscriber dronePoseSubs;
    void dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);


    //drone point to look (subscriber)
private:
    droneMsgsROS::dronePositionRefCommand dronePointToLookMsg;
    ros::Subscriber dronePointToLookSub;
    void dronePointToLookCallback(const droneMsgsROS::dronePositionRefCommand::ConstPtr& msg);


    //yaw to look (subscriber)
private:
    droneMsgsROS::droneYawRefCommand droneYawToLookMsg;
    ros::Subscriber droneYawToLookSub;
    void droneYawToLookCallback(const droneMsgsROS::droneYawRefCommand::ConstPtr& msg);



    //yaw command to the controller (publisher)
private:
    droneMsgsROS::droneYawRefCommand droneYawRefCommandMsg;
    ros::Publisher droneYawRefCommandPub;
    int publishYawRefCommand(droneMsgsROS::droneYawRefCommand droneYawRefCommandIn);



public:
    DroneYawCommander();
    ~DroneYawCommander();
	
    void open(ros::NodeHandle & nIn, std::string moduleName);
	void close();

protected:
    bool init();


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
