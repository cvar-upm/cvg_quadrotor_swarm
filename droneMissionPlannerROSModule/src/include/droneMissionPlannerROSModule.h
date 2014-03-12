/*
 * 
 *
 *  Created on: 
 *      Author: Jose Luis Sanchez-Lopez
 */

#ifndef DRONE_MISSION_PLANNER_H
#define DRONE_MISSION_PLANNER_H



////// ROS  ///////
#include "ros/ros.h"



//Math
#include <math.h>


#include <cstdlib>
#include <ctime>



#include <iostream>




//Drone module
#include "droneModuleROS.h"


//Trajectory planner
#include "missionPlanner.h"



////Msgs
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneHLCommand.h"
#include "droneMsgsROS/droneHLCommandAck.h"
#include "droneMsgsROS/dronePositionRefCommand.h"
#include "droneMsgsROS/droneYawRefCommand.h"

#include "droneMsgsROS/droneMissionInfo.h"
#include "droneMsgsROS/droneGoTask.h"


////services
#include "std_srvs/Empty.h"


//Communications
#include "communication_definition.h"


//Freq
#define FREQ_MISSION_PLANNER    10.0






class DroneMissionPlanner : public DroneModule
{	

private:
    //mission planner
    RobotMissionPlanner MyMissionPlanner;

    //counters
    int submissionCounter;
    int taskCounter;

    //flags
    bool flagMissionEnded;
    //bool flagNextSubmission;
    bool flagTaskEnded;

    bool flagNewTask;
    bool flagSendNewTask;
    bool flagNewSubmission;
    bool flagUpdateSubmissionTime;

    bool flagAckReceived;

    bool flagTaskReady;

    bool flagTaskRunning;

    bool flagFinishSubmissionLoop;
    bool flagFinishTaskLoop;

    bool flagModuleSuspended;


/////Communications!!

    //drone pose
private:
    droneMsgsROS::dronePose dronePoseMsg;
    /////Subscribers
    ros::Subscriber dronePoseSub;
    void dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);

    //drone yaw command (subscriber)
private:
    droneMsgsROS::droneYawRefCommand droneYawRefCommandMsg;
    /////Subscribers
    ros::Subscriber droneYawRefCommandSub;
    void droneYawRefCommandCallback(const droneMsgsROS::droneYawRefCommand::ConstPtr& msg);


    //drone HL command (publisher)
private:
    droneMsgsROS::droneHLCommand droneHLCommMsg;
    ros::Publisher droneHLCommPub;
    int publishHLCommand(droneMsgsROS::droneHLCommand droneHLCommMsgIn);


    //drone HL command ack (subs)
private:
    droneMsgsROS::droneHLCommandAck droneHLCommAckMsg;
    ros::Subscriber droneHLCommAckSub;
    void droneHLCommandAckCallback(const droneMsgsROS::droneHLCommandAck::ConstPtr& msg);


    //drone Mission point (publisher)
private:
    //JL TODO poner
    //#include "geometry_msgs/Vector3Stamped.h"
    droneMsgsROS::dronePositionRefCommand dronePositionRefCommandMsg;
    ros::Publisher dronePositionRefCommandPub;
    int publishPositionRefCommand(droneMsgsROS::dronePositionRefCommand dronePositionRefCommandIn);


    //yaw to look (publisher)
private:
    droneMsgsROS::droneYawRefCommand droneYawToLookMsg;
    ros::Publisher droneYawToLookPub;
    int publishYawToLook(droneMsgsROS::droneYawRefCommand droneYawToLookIn);


    //point to look (publisher)
private:
    //JL TODO poner
    //#include "geometry_msgs/Vector3Stamped.h"
    droneMsgsROS::dronePositionRefCommand dronePointToLookMsg;
    ros::Publisher dronePointToLookPub;
    int publishPointToLook(droneMsgsROS::dronePositionRefCommand dronePointToLookIn);


    //drone mission info (publisher)
private:
    droneMsgsROS::droneMissionInfo droneMissionInfoMsg;
    ros::Publisher droneMissionInfoPub;
    int publishMissionInfo(droneMsgsROS::droneMissionInfo droneMissionInfoIn);



    //go to task (subscriber)
private:
    droneMsgsROS::droneGoTask droneGoTaskMsg;
    ros::Subscriber droneGoTaskSub;
    void droneGoTaskCallback(const droneMsgsROS::droneGoTask::ConstPtr& msg);


    //empty service
private:
    std_srvs::Empty emptySrv;


    //suspend (service)
private:
    ros::ServiceServer suspendServerSrv;
    bool suspendServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    //resume (service)
private:
    ros::ServiceServer resumeServerSrv;
    bool resumeServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);




//// Others
    //durations
protected:
    ros::Time missionInitTime;
    ros::Time submissionInitTime;
    ros::Time taskInitTime;


    ros::Time suspendedInitTime;
    ros::Duration supendedTime;







public:
    DroneMissionPlanner();
    ~DroneMissionPlanner();
	
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
