/*
 * 
 *
 *  Created on: 
 *      Author: Jose Luis Sanchez-Lopez
 */


#include "droneMissionPlannerROSModule.h"



using namespace std;




DroneMissionPlanner::DroneMissionPlanner() : DroneModule(droneModule::active,FREQ_MISSION_PLANNER)
{
    if(!init())
        cout<<"Error init"<<endl;


    return;
}



DroneMissionPlanner::~DroneMissionPlanner()
{
	close();
	return;
}

void DroneMissionPlanner::open(ros::NodeHandle & nIn, std::string moduleName)
{
	//Node
    DroneModule::open(nIn,moduleName);



    //Set mission
    MyMissionPlanner.setMission(stackPath+"configs/drone"+stringId+"/configMission.xml");


    //Init mission
    submissionCounter=MyMissionPlanner.submissionTree.at(0);
    taskCounter=MyMissionPlanner.taskTree.at(0).at(0);

    cout<<" .init submission="<<submissionCounter<<endl;
    cout<<" .init task="<<taskCounter<<endl;


	
    //// Topics ///
    ///subs
    // rostopic pub -1 /drone1/ArucoSlam_EstimatedPose droneMsgsROS/dronePose -- 123 0.0 0.0 1.5 0.0 0.0 0.0 "a" "a" "a"
    dronePoseSub = n.subscribe(DRONE_MISSION_PLANNER_POSE_SUBSCRIPTION, 1, &DroneMissionPlanner::dronePoseCallback, this);
    // rostopic pub -1 /drone1/droneMissionHLCommandAck droneMsgsROS/droneHLCommandAck -- 123 true
    droneHLCommAckSub = n.subscribe(DRONE_MISSION_PLANNER_HL_COMMAND_ACK, 1, &DroneMissionPlanner::droneHLCommandAckCallback, this);

    droneGoTaskSub = n.subscribe(DRONE_MISSION_GO_TASK_SUBS, 1, &DroneMissionPlanner::droneGoTaskCallback, this);

    droneYawRefCommandSub = n.subscribe(DRONE_MISSION_YAW_REF_CMD_SUBS, 1, &DroneMissionPlanner::droneYawRefCommandCallback, this);


    ///pub
    droneHLCommPub=n.advertise<droneMsgsROS::droneHLCommand>(DRONE_MISSION_PLANNER_HL_COMMAND, 1, true);
    dronePositionRefCommandPub=n.advertise<droneMsgsROS::dronePositionRefCommand>(DRONE_MISSION_PLANNER_MISSION_POINT_REF, 1, true);
    droneYawToLookPub=n.advertise<droneMsgsROS::droneYawRefCommand>(DRONE_MISSION_PLANNER_YAW_TO_LOOK_PUB, 1, true);
    dronePointToLookPub=n.advertise<droneMsgsROS::dronePositionRefCommand>(DRONE_MISSION_PLANNER_POINT_TO_LOOK_PUB, 1, true);
    droneMissionInfoPub=n.advertise<droneMsgsROS::droneMissionInfo>(DRONE_MISSION_INFO_PUB, 1, true);



    //services
    suspendServerSrv=n.advertiseService(moduleName+"/suspend",&DroneMissionPlanner::suspendServCall,this);
    resumeServerSrv=n.advertiseService(moduleName+"/resume",&DroneMissionPlanner::resumeServCall,this);



    //Flag of module opened
    droneModuleOpened=true;


    //Start TODO JL_to remove!!
    //startVal();

	
	//End
	return;
}



void DroneMissionPlanner::dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg)
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


void DroneMissionPlanner::droneYawRefCommandCallback(const droneMsgsROS::droneYawRefCommand::ConstPtr& msg)
{
    droneYawRefCommandMsg.header=msg->header;
    droneYawRefCommandMsg.yaw=msg->yaw;

    return;
}


void DroneMissionPlanner::droneHLCommandAckCallback(const droneMsgsROS::droneHLCommandAck::ConstPtr& msg)
{
    droneHLCommAckMsg.time=msg->time;
    droneHLCommAckMsg.ack=msg->ack;

    flagAckReceived=droneHLCommAckMsg.ack;

    //Trye to resend again
    if(!flagAckReceived)
        flagSendNewTask=true;

    return;
}


void DroneMissionPlanner::droneGoTaskCallback(const droneMsgsROS::droneGoTask::ConstPtr& msg)
{
    //read the msgs
    droneGoTaskMsg.time=msg->time;
    droneGoTaskMsg.submissionId=msg->submissionId;
    droneGoTaskMsg.taskId=msg->taskId;

    //do things
    resetValues();
    submissionCounter=droneGoTaskMsg.submissionId;
    taskCounter=droneGoTaskMsg.taskId;



    //end
    return;
}


int DroneMissionPlanner::publishHLCommand(droneMsgsROS::droneHLCommand droneHLCommMsgIn)
{
    if(droneModuleOpened==false)
        return 0;

    droneHLCommPub.publish(droneHLCommMsgIn);

    return 1;
}


int DroneMissionPlanner::publishPositionRefCommand(droneMsgsROS::dronePositionRefCommand dronePositionRefCommandIn)
{
    if(droneModuleOpened==false)
        return 0;

    dronePositionRefCommandPub.publish(dronePositionRefCommandIn);

    return 1;
}

int DroneMissionPlanner::publishYawToLook(droneMsgsROS::droneYawRefCommand droneYawToLookIn)
{
    if(droneModuleOpened==false)
        return 0;

    droneYawToLookPub.publish(droneYawToLookIn);

    return 1;
}


int DroneMissionPlanner::publishPointToLook(droneMsgsROS::dronePositionRefCommand dronePointToLookIn)
{
    if(droneModuleOpened==false)
        return 0;

    dronePointToLookPub.publish(dronePointToLookIn);

    return 1;
}


int DroneMissionPlanner::publishMissionInfo(droneMsgsROS::droneMissionInfo droneMissionInfoIn)
{
    if(droneModuleOpened==false)
        return 0;

    droneMissionInfoPub.publish(droneMissionInfoIn);

    return 1;
}


bool DroneMissionPlanner::suspendServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(flagModuleSuspended)
        return false;

    cout<<"Suspending!!"<<endl;

    flagModuleSuspended=true;

    suspendedInitTime=ros::Time::now();


    return true;
}


bool DroneMissionPlanner::resumeServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(!flagModuleSuspended)
        return false;

    cout<<"Resuming!!"<<endl;

    flagModuleSuspended=false;


    supendedTime+=ros::Time::now()-suspendedInitTime;

    return true;
}


bool DroneMissionPlanner::init()
{
    //counters
    submissionCounter=0;
    taskCounter=0;

    flagMissionEnded=false;
    flagTaskEnded=false;

    flagNewTask=true;
    flagSendNewTask=false;
    flagNewSubmission=true;
    flagUpdateSubmissionTime=true;

    flagAckReceived=false;

    flagTaskReady=false;
    flagTaskRunning=false;

    flagFinishSubmissionLoop=false;
    flagFinishTaskLoop=false;

    flagModuleSuspended=false;

    //end
    return true;
}



void DroneMissionPlanner::close()
{

    DroneModule::close();
	return;
}



bool DroneMissionPlanner::resetValues()
{
    //Init mission
    missionInitTime=ros::Time::now();
    submissionInitTime=ros::Time::now();
    taskInitTime==ros::Time::now();

    submissionCounter=MyMissionPlanner.submissionTree.at(0);
    taskCounter=MyMissionPlanner.taskTree.at(0).at(0);

    flagMissionEnded=false;
    flagTaskEnded=false;

    flagNewTask=true;
    flagSendNewTask=false;
    flagNewSubmission=true;
    flagUpdateSubmissionTime=true;

    flagAckReceived=false;

    flagTaskReady=false;
    flagTaskRunning=false;

    flagFinishSubmissionLoop=false;
    flagFinishTaskLoop=false;

    flagModuleSuspended=false;


    return true;

}



bool DroneMissionPlanner::startVal()
{
    //Reset time

    //timeflagModuleSuspended;
    missionInitTime=ros::Time::now();


    //Read task
    if(!MyMissionPlanner.readTask(submissionCounter,taskCounter))
    {
        cout<<" .Submission or task not found!"<<endl;
        return false;
    }
    submissionInitTime=ros::Time::now();
    taskInitTime==ros::Time::now();


    //End
    return DroneModule::startVal();
}



bool DroneMissionPlanner::stopVal()
{
    return DroneModule::stopVal();
}



bool DroneMissionPlanner::run()
{
    if(!DroneModule::run())
        return false;

    if(droneModuleOpened==false)
        return false;

    if(flagModuleSuspended)
        return false;


    cout<<"New loop"<<endl;


    //Analize ack
    if(flagAckReceived)
    {
        cout<<"-Ack received!"<<endl;
        //Bajar bandera
        flagAckReceived=false;

        //Check actual task
        if(MyMissionPlanner.TheTask.type==missionPlanner::takeOff || MyMissionPlanner.TheTask.type==missionPlanner::land)
        {
            cout<<"  .task unable to be monitored"<<endl;
            flagTaskEnded=true;
        }
        else
        {
            cout<<"  .task to be monitored"<<endl;
            flagTaskReady=true;
        }

    }



    //check if task ended
    if(flagTaskEnded)
    {
        cout<<"-TaskEnded"<<endl;

        //bajamos bandera
        flagTaskEnded=false;


        //Check if the task is periodic
        if(MyMissionPlanner.TheTask.loop==true && !flagFinishTaskLoop)
        {
            //reset some parameters
            // TODO JL
            //taskInitTime=ros::Time::now();

            flagNewTask=true;
        }
        else
        {
            //bajamos flag si procede
            if(flagFinishTaskLoop)
                flagFinishTaskLoop=false;

            //search
            unsigned int iSubm;
            for(iSubm=0;iSubm<MyMissionPlanner.submissionTree.size();iSubm++)
            {
                if(MyMissionPlanner.submissionTree.at(iSubm)==submissionCounter)
                    break;
            }
            //cout<<"iSubm="<<iSubm<<endl;
            unsigned int iTask;
            for(iTask=0;iTask<MyMissionPlanner.taskTree.at(iSubm).size();iTask++)
            {
                if(MyMissionPlanner.taskTree.at(iSubm).at(iTask)==taskCounter)
                    break;
            }
            //cout<<"iTask="<<iTask<<endl;
            //read new task
            //cout<<"MyMissionPlanner.taskTree.at(iSubm).size()="<<MyMissionPlanner.taskTree.at(iSubm).size()<<endl;
            if(iTask<MyMissionPlanner.taskTree.at(iSubm).size()-1)
            {
                //There are more tasks in the submission
                //cout<<"There are more tasks in the submission"<<endl;
                taskCounter=MyMissionPlanner.taskTree.at(iSubm).at(iTask+1);

                //taskInitTime=ros::Time::now();

                flagNewTask=true;
                //cout<<"taskCounter="<<taskCounter<<endl;
            }
            else
            {
                //Check if the submission is periodic
                if(MyMissionPlanner.TheSubmission.loop && !flagFinishSubmissionLoop)
                {
                    //reset some parameters
                    //submissionInitTime=ros::Time::now();

                    taskCounter=MyMissionPlanner.taskTree.at(iSubm).at(0);

                    //taskInitTime=ros::Time::now();
                    flagNewSubmission=true;
                    flagUpdateSubmissionTime=false;
                    flagNewTask=true;
                }
                else
                {
                    //bajamos flag si procede
                    if(flagFinishSubmissionLoop)
                    {
                        cout<<"  .!!!Finish Submission Loop!!!!"<<endl;
                        flagFinishSubmissionLoop=false;
                    }

                    //New submission
                    if(iSubm<MyMissionPlanner.submissionTree.size()-1)
                    {
                        //cout<<"  .New submission"<<endl;
                        submissionCounter=MyMissionPlanner.submissionTree.at(iSubm+1);
                        //cout<<"submissionCounter="<<submissionCounter<<endl;
                        taskCounter=MyMissionPlanner.taskTree.at(iSubm+1).at(0);
                        //cout<<"taskCounter="<<taskCounter<<endl;

                        //submissionInitTime=ros::Time::now();
                        //taskInitTime=ros::Time::now();

                        flagNewTask=true;
                        flagNewSubmission=true;
                        flagUpdateSubmissionTime=true;
                    }
                    else
                    {
                        //we finished the mission
                        flagMissionEnded=true;
                    }
                }
            }
        }

    }

    //Check if mission ended
    if(flagMissionEnded)
    {
        cout<<"-Mission ended!"<<endl;
        return false;
    }



    //New task. communicate to the brain!
    if(flagNewTask)
    {
        //bajar bandera
        flagNewTask=false;

        cout<<"-!New task!. submission="<<submissionCounter<<"; task="<<taskCounter<<endl;


        //command task!
        if(!MyMissionPlanner.readTask(submissionCounter,taskCounter))
        {
            cout<<"  .Submission or task not found!"<<endl;
            return false;
        }


        flagSendNewTask=true;
    }


    if(flagSendNewTask)
    {
        //bajamos badera
        flagSendNewTask=false;

        cout<<"-send new task!!"<<endl;

        //send
        droneHLCommMsg.time=ros::Time::now();

        if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::takeOff)
        {
            droneHLCommMsg.hlCommand=droneMsgsROS::droneHLCommand::TAKE_OFF;
        }
        else if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::hover)
        {
            droneHLCommMsg.hlCommand=droneMsgsROS::droneHLCommand::HOVER;
        }
        else if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::land)
        {
            droneHLCommMsg.hlCommand=droneMsgsROS::droneHLCommand::LAND;
        }
        else if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::trajectoryMovement)
        {
            droneHLCommMsg.hlCommand=droneMsgsROS::droneHLCommand::MOVE_TRAJECTORY;
        }
        else if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::sleep)
        {
            droneHLCommMsg.hlCommand=droneMsgsROS::droneHLCommand::SLEEP;
        }
        else
        {
            droneHLCommMsg.hlCommand=droneMsgsROS::droneHLCommand::UNKNOWN;
        }

        //publicamos
        publishHLCommand(droneHLCommMsg);

    }


    //launch task, because it is ready (the brain says ok)
    bool flagMonitorMissionNow=true;
    if(flagTaskReady)
    {

        cout<<"-The task is ready!"<<endl;

        //bajamos bandera
        flagTaskReady=false;

        //subimos la otra
        flagTaskRunning=true;


        //Init task
        taskInitTime=ros::Time::now();
        //cout<<"    setting task time to "<<taskInitTime<<endl;

        if(flagNewSubmission)
        {
            cout<<"  .!!New submission!!"<<endl;

            //bajamos bandera
            flagNewSubmission=false;

            if(flagUpdateSubmissionTime)
            {
                flagUpdateSubmissionTime=false;

                cout<<"  .!!updating time!"<<endl;
                submissionInitTime=ros::Time::now();
            }
        }


        //launch task!
        if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::trajectoryMovement)
        {
            //Point!!!
            bool pointLoaded=false;
            //x
            if(MyMissionPlanner.TheTask.pointLoaded.at(0))
            {
                pointLoaded=true;
                dronePositionRefCommandMsg.x=MyMissionPlanner.TheTask.point.at(0);
            }
            else
            {
                dronePositionRefCommandMsg.x=dronePoseMsg.x;
                MyMissionPlanner.TheTask.point.at(0)=dronePoseMsg.x;
            }

            //y
            if(MyMissionPlanner.TheTask.pointLoaded.at(1))
            {
                pointLoaded=true;
                dronePositionRefCommandMsg.y=MyMissionPlanner.TheTask.point.at(1);
            }
            else
            {
                dronePositionRefCommandMsg.y=dronePoseMsg.y;
                MyMissionPlanner.TheTask.point.at(1)=dronePoseMsg.y;
            }

            //z
            if(MyMissionPlanner.TheTask.pointLoaded.at(2))
            {
                pointLoaded=true;
                dronePositionRefCommandMsg.z=MyMissionPlanner.TheTask.point.at(2);
            }
            else
            {
                dronePositionRefCommandMsg.z=dronePoseMsg.z;
                MyMissionPlanner.TheTask.point.at(2)=dronePoseMsg.z;
            }

            //enviamos el punto
            if(pointLoaded)
                publishPositionRefCommand(dronePositionRefCommandMsg);


            /////Yaw. Always send a command
            //Yaw
            if(MyMissionPlanner.TheTask.yawLoaded)
            {
                //command from xml
                droneYawToLookMsg.yaw=MyMissionPlanner.TheTask.yaw;
                publishYawToLook(droneYawToLookMsg);
            }
            /*
            else
            {
                //same yaw than before
                droneYawToLookMsg.yaw=dronePoseMsg.yaw;
                publishYawToLook(droneYawToLookMsg);
            }
            */

            //Point to look
            if(MyMissionPlanner.TheTask.pointToLookLoaded.at(0) && MyMissionPlanner.TheTask.pointToLookLoaded.at(1))
            {
                //comand from xml
                dronePointToLookMsg.x=MyMissionPlanner.TheTask.pointToLook.at(0);
                dronePointToLookMsg.y=MyMissionPlanner.TheTask.pointToLook.at(1);
                publishPointToLook(dronePointToLookMsg);
            }
            /*
            else
            {
                //same yaw than before
                droneYawToLookMsg.yaw=dronePoseMsg.yaw;
                publishYawToLook(droneYawToLookMsg);
            }
            */


        }

        flagMonitorMissionNow=false;

    }



    //////Monitor mission!!
    if(flagTaskRunning && flagMonitorMissionNow)
    {
        //Monitor the submission
        cout<<"-Monitoring mission"<<endl;

        //Time
        if(MyMissionPlanner.TheSubmission.duration!=-1)
        {
            cout<<"  .periodic submision"<<endl;

            //There is a limit in time
            ros::Duration submissionDuration;
            submissionDuration=ros::Time::now()-submissionInitTime-supendedTime;

            cout<<"    ->time="<<submissionDuration.toSec()<<" of "<<MyMissionPlanner.TheSubmission.duration<<endl;

            if(submissionDuration.toSec()>=MyMissionPlanner.TheSubmission.duration)
            {
                if(MyMissionPlanner.TheSubmission.loop)
                    flagFinishSubmissionLoop=true;
            }
        }

        //battery
        /*
        if(MyMissionPlanner.TheSubmission.battery!=-1)
        {
            //There is a limit in battery

        }
        */


        //monitor de task
        cout<<"  .Monitoring the task"<<endl;

        //move
        if(MyMissionPlanner.TheTask.type==missionPlanner::taskTypes::trajectoryMovement)
        {
            cout<<"  .Moving!"<<endl;

            //time. No sense
            /*
            if(MyMissionPlanner.TheTask.duration!=-1)
            {
                //There is a limit in time
                ros::Duration taskDuration;
                taskDuration=taskInitTime-ros::Time::now();

                if(taskDuration.toSec()>=MyMissionPlanner.TheTask.duration)
                {
                    if(MyMissionPlanner.TheTask.loop)
                        flagFinishTaskLoop=true;
                }
            }
            */


            //battery. No sense


            //mission achieved
            /*
            double distanceToMissionPoint=0.0;
            distanceToMissionPoint+=pow(dronePoseMsg.x-MyMissionPlanner.TheTask.point.at(0),2);
            distanceToMissionPoint+=pow(dronePoseMsg.y-MyMissionPlanner.TheTask.point.at(1),2);
            distanceToMissionPoint+=pow(dronePoseMsg.z-MyMissionPlanner.TheTask.point.at(2),2);
            distanceToMissionPoint=sqrt(distanceToMissionPoint);
            cout<<"  ->distances="<<distanceToMissionPoint<<endl;
            */


            //cout<<"pose=["<<dronePoseMsg.x<<"; "<<dronePoseMsg.y<<"; "<<dronePoseMsg.z<<"]"<<endl;
            //cout<<"mission point=["<<MyMissionPlanner.TheTask.point.at(0)<<"; "<<MyMissionPlanner.TheTask.point.at(1)<<"; "<<MyMissionPlanner.TheTask.point.at(2)<<"]"<<endl;


            //Position distances!!! Always important!!
            //TODO_JL fix this!! now is a square in x-y. it should be a circle!!
            std::vector<double> distanceToMissionPoint;
            //x
            distanceToMissionPoint.push_back(fabs(dronePoseMsg.x-MyMissionPlanner.TheTask.point.at(0)));
            //y
            distanceToMissionPoint.push_back(fabs(dronePoseMsg.y-MyMissionPlanner.TheTask.point.at(1)));
            //z
            distanceToMissionPoint.push_back(fabs(dronePoseMsg.z-MyMissionPlanner.TheTask.point.at(2)));

            cout<<"  ->distances="<<distanceToMissionPoint.at(0)<<"; "<<distanceToMissionPoint.at(1)<<"; "<<distanceToMissionPoint.at(2)<<endl;


            ///Yaw!!
            //yaw distance
            double distanceToYaw=0.0;

            distanceToYaw=fabs(dronePoseMsg.yaw-droneYawRefCommandMsg.yaw);

            cout<<"  ->distance yaw="<<distanceToYaw*180.0/M_PI<<endl;

            /*
            if(MyMissionPlanner.TheTask.pointToLookLoaded.at(0) && MyMissionPlanner.TheTask.pointToLookLoaded.at(1))
            {
                //Point to look distance
                //TODO_JL
            }
            else
            {
                //A command is always sent
                //Yaw command
                //if(MyMissionPlanner.TheTask.yawLoaded)
                    distanceToYaw=fabs(dronePoseMsg.yaw-MyMissionPlanner.TheTask.yaw);
            }
            */


            //Check distances
            bool flagPositionAchieved=false;
            if(distanceToMissionPoint.at(0)<=MyMissionPlanner.defaultPrecission.at(0) && distanceToMissionPoint.at(1)<=MyMissionPlanner.defaultPrecission.at(1) && distanceToMissionPoint.at(2)<=MyMissionPlanner.defaultPrecission.at(2))
            {
                flagPositionAchieved=true;
            }

            //Check yaw distances
            bool flagYawAchieved=false;
            if(distanceToYaw<=MyMissionPlanner.yawPrecission)
            {
                flagYawAchieved=true;
            }



            //Finish mission
            if(flagPositionAchieved && flagYawAchieved)
            {
                flagTaskEnded=true;
                flagTaskRunning=false;
            }


        }

        //hover
        if(MyMissionPlanner.TheTask.type==missionPlanner::hover)
        {
            cout<<"  .Hovering!"<<endl;

            //time
            if(MyMissionPlanner.TheTask.duration!=-1)
            {
                //There is a limit in time
                ros::Duration taskDuration;
                taskDuration=ros::Time::now()-taskInitTime-supendedTime;

                //cout<<"     -init time task="<<taskInitTime<<endl;
                //cout<<"     -ros now="<<ros::Time::now()<<endl;

                cout<<"   ->time="<<taskDuration.toSec()<<endl;

                if(taskDuration.toSec()>=MyMissionPlanner.TheTask.duration)
                {
                    flagTaskEnded=true;
                    flagTaskRunning=false;
                }
            }

            //battery. No sense

        }

        //sleep
        if(MyMissionPlanner.TheTask.type==missionPlanner::sleep)
        {
            cout<<"  .Sleeping!"<<endl;

            //time
            if(MyMissionPlanner.TheTask.duration!=-1)
            {
                //There is a limit in time
                ros::Duration taskDuration;
                taskDuration=ros::Time::now()-taskInitTime-supendedTime;

                cout<<"   ->time="<<taskDuration.toSec()<<endl;

                if(taskDuration.toSec()>=MyMissionPlanner.TheTask.duration)
                {
                    flagTaskEnded=true;
                    flagTaskRunning=false;
                }
            }

            //battery. No sense

        }
    }


    //Mission info

    ////set the variable
    droneMissionInfoMsg.timeMsgs=ros::Time::now();
    //missiom
    droneMissionInfoMsg.durationMission=ros::Time::now()-missionInitTime-supendedTime;
    //subtask
    droneMissionInfoMsg.durationSubmission=ros::Time::now()-submissionInitTime-supendedTime;
    droneMissionInfoMsg.idSubmission=submissionCounter;
    droneMissionInfoMsg.loopSubmission=MyMissionPlanner.TheSubmission.loop;
    //task
    droneMissionInfoMsg.durationTask=ros::Time::now()-taskInitTime-supendedTime;
    droneMissionInfoMsg.idTask=taskCounter;
    if(MyMissionPlanner.TheTask.type==missionPlanner::hover)
        droneMissionInfoMsg.taskType=droneMsgsROS::droneHLCommand::HOVER;
    else if(MyMissionPlanner.TheTask.type==missionPlanner::land)
        droneMissionInfoMsg.taskType=droneMsgsROS::droneHLCommand::LAND;
    else if(MyMissionPlanner.TheTask.type==missionPlanner::takeOff)
        droneMissionInfoMsg.taskType=droneMsgsROS::droneHLCommand::TAKE_OFF;
    else if(MyMissionPlanner.TheTask.type==missionPlanner::sleep)
        droneMissionInfoMsg.taskType=droneMsgsROS::droneHLCommand::SLEEP;
    else if(MyMissionPlanner.TheTask.type==missionPlanner::trajectoryMovement)
        droneMissionInfoMsg.taskType=droneMsgsROS::droneHLCommand::MOVE_TRAJECTORY;
    else
        droneMissionInfoMsg.taskType=droneMsgsROS::droneHLCommand::UNKNOWN;
    if(flagTaskRunning)
        droneMissionInfoMsg.taskState=droneMsgsROS::droneMissionInfo::TASK_RUNNING;
    else
        droneMissionInfoMsg.taskState=droneMsgsROS::droneMissionInfo::WAITING_BRAIN;



    ////publish
    publishMissionInfo(droneMissionInfoMsg);





    return false;
}


