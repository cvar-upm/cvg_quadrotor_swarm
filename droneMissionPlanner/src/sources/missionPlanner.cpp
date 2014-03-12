/*
 * 
 *
 *  Created on: 
 *      Author: Jose Luis Sanchez-Lopez
 */


#include "missionPlanner.h"

using namespace std;



int Submission::readParameters(const pugi::xml_node &submission)
{
    //Flush values
    id=-1;
    loop=false;
    duration=-1;
    battery=-1;

    //Read parameters
    string readingValue;
    //id
    readingValue=submission.child("config").child_value("id");
    istringstream convertidSubmision(readingValue);
    convertidSubmision>>id;
    //loop
    readingValue=submission.child("config").child_value("loop");
    istringstream convertloopSubmision(readingValue);
    convertloopSubmision>>loop;
    //duration
    readingValue=submission.child("config").child_value("time");
    istringstream convertdurationSubmision(readingValue);
    convertdurationSubmision>>duration;
    //battery
    readingValue=submission.child("config").child_value("battery");
    istringstream convertbatterySubmision(readingValue);
    convertbatterySubmision>>duration;

    return 1;
}


int Task::readParameters(const pugi::xml_node &task)
{
    ////Flush value

    //point to move
    point.clear();
    point.resize(3);
    pointLoaded.resize(3);
    for(unsigned int i=0;i<pointLoaded.size();i++)
        pointLoaded.at(i)=false;

    //Yaw
    yawLoaded=false;

    pointToLook.clear();
    pointToLook.resize(2);
    pointToLookLoaded.resize(2);
    for(unsigned int i=0;i<pointToLookLoaded.size();i++)
        pointToLookLoaded.at(i)=false;

    //mission type
    type=missionPlanner::unknown;


    //// read Common parameters
    Submission::readParameters(task);

    //specific parameters
    string readingValue;

    //command = task type
    readingValue=task.child_value("command");

    if(readingValue==CMD_TAKEOFF)
    {
        type=missionPlanner::takeOff;
    }
    else if(readingValue==CMD_HOVER)
    {
        type=missionPlanner::hover;
    }
    else if(readingValue==CMD_LAND)
    {
        type=missionPlanner::land;
    }
    else if(readingValue==CMD_TRAJ_MOVEMENT)
    {
        type=missionPlanner::trajectoryMovement;
    }
    else if(readingValue==CMD_SLEEP)
    {
        type=missionPlanner::sleep;
    }
    else
    {
        type=missionPlanner::unknown;
    }


    //point mission
    //x
    readingValue=task.child("point").child_value("x");
    if(readingValue!="")
    {
        pointLoaded.at(0)=true;
        istringstream convertxtask(readingValue);
        convertxtask>>point.at(0);
    }

    //y
    readingValue=task.child("point").child_value("y");
    if(readingValue!="")
    {
        pointLoaded.at(1)=true;
        istringstream convertytask(readingValue);
        convertytask>>point.at(1);
    }

    //z
    readingValue=task.child("point").child_value("z");
    if(readingValue!="")
    {
        pointLoaded.at(2)=true;
        istringstream convertztask(readingValue);
        convertztask>>point.at(2);
    }


    //yaw
    readingValue=task.child_value("yaw");
    if(readingValue!="")
    {
        yawLoaded=true;
        istringstream convertyawSubmision(readingValue);
        convertyawSubmision>>yaw;
        yaw*=(M_PI/180.0);
    }


    //point to look
    //x
    readingValue=task.child("pointToLook").child_value("x");
    if(readingValue!="")
    {
        pointToLookLoaded.at(0)=true;
        istringstream convertxtask(readingValue);
        convertxtask>>pointToLook.at(0);
    }

    //y
    readingValue=task.child("pointToLook").child_value("y");
    if(readingValue!="")
    {
        pointToLookLoaded.at(1)=true;
        istringstream convertytask(readingValue);
        convertytask>>pointToLook.at(1);
    }

    //if both loaded, we discard yaw loaded!!
    if(pointToLookLoaded.at(0) && pointToLookLoaded.at(1) && yawLoaded)
        yawLoaded=false;


    return 1;
}


RobotMissionPlanner::RobotMissionPlanner()
{
    init();
    return;
}

RobotMissionPlanner::~RobotMissionPlanner()
{
    clear();
    return;
}


int RobotMissionPlanner::init()
{
    return 1;
}

int RobotMissionPlanner::clear()
{
    return 1;
}




int RobotMissionPlanner::setMission(std::string missionConfigFile)
{
    //xml parser

    std::ifstream nameFile(missionConfigFile);
    pugi::xml_parse_result result = doc.load(nameFile);
    if(!result)
        return 0;

    cout<<" .opened "<<missionConfigFile<<endl;


    //read general parameters of the mission
    pugi::xml_node config=doc.child("mission").child("config");
    std::string readingValue;

    defaultPrecission.resize(3);


    //x
    readingValue=config.child("precission").child_value("x");
    if(readingValue!="")
    {
        istringstream convertxprecission(readingValue);
        convertxprecission>>defaultPrecission.at(0);
    }

    //y
    readingValue=config.child("precission").child_value("y");
    if(readingValue!="")
    {
        istringstream convertyprecission(readingValue);
        convertyprecission>>defaultPrecission.at(1);
    }

    //z
    readingValue=config.child("precission").child_value("z");
    if(readingValue!="")
    {
        istringstream convertzprecission(readingValue);
        convertzprecission>>defaultPrecission.at(2);
    }

    //yaw
    readingValue=config.child("precission").child_value("yaw");
    if(readingValue!="")
    {
        istringstream convertzprecission(readingValue);
        convertzprecission>>yawPrecission;
    }
    else
        yawPrecission=10.0;


    yawPrecission*=M_PI/180.0;



    if(!generateMissionTree())
        return 0;


    //end
    return 1;
}


int RobotMissionPlanner::generateMissionTree()
{
    //clear
    submissionTree.clear();
    taskTree.clear();


    //aux vars
    std::vector<int> tasksInSubmission;
    int readId;
    string readingValue;


    //generate submission tree
    cout<<" .generating tree"<<endl;
    pugi::xml_node submission;
    for(submission = doc.child("mission").child("submission"); submission; submission = submission.next_sibling("submission"))
    {
        //read submission id
        readingValue=submission.child("config").child_value("id");
        istringstream convertidSubmision(readingValue);
        convertidSubmision>>readId;

        //add id of the submission to the submission tree
        submissionTree.push_back(readId);


    }
    //sort
    std::sort(submissionTree.begin(),submissionTree.end());


    //generate task tree
    for(unsigned int i=0;i<submissionTree.size();i++)
    {

        cout<<"  ->submission="<<submissionTree.at(i)<<endl;

        //search the submission using id
        bool flagSubmissionFound=false;
        for(submission = doc.child("mission").child("submission"); submission; submission = submission.next_sibling("submission"))
        {
            //read submission id
            int readId;
            string readingValue=submission.child("config").child_value("id");
            istringstream convertidSubmision(readingValue);
            convertidSubmision>>readId;
            if(readId==submissionTree.at(i))
            {
                flagSubmissionFound=true;
                //cout<<"  read id="<<readId<<endl;
                break;
            }
        }

        if(flagSubmissionFound)
        {
            //read tasks
            pugi::xml_node task;
            tasksInSubmission.clear();
            for(task = submission.child("task"); task; task = task.next_sibling("task"))
            {
                //read task id
                readingValue=task.child("config").child_value("id");
                istringstream convertidTask(readingValue);
                convertidTask>>readId;

                tasksInSubmission.push_back(readId);



            }

            //sort
            std::sort(tasksInSubmission.begin(),tasksInSubmission.end());

            for(unsigned int j=0;j<tasksInSubmission.size();j++)
            {
                cout<<"   ->task="<<tasksInSubmission.at(j)<<endl;
            }

            //push back
            taskTree.push_back(tasksInSubmission);
        }

    }


    return 1;
}


int RobotMissionPlanner::readTask(int submissionIdIn, int taskIdIn)
{

    //search submission
    pugi::xml_node submission;
    bool flagSubmissionFound=false;
    for(submission = doc.child("mission").child("submission"); submission; submission = submission.next_sibling("submission"))
    {
        //read submission id
        int readId;
        string readingValue=submission.child("config").child_value("id");
        istringstream convertidSubmision(readingValue);
        convertidSubmision>>readId;
        if(readId==submissionIdIn)
        {
            flagSubmissionFound=true;
            //cout<<"  read id="<<readId<<endl;
            break;
        }
    }


    //search task
    pugi::xml_node task;
    bool flagTaskFound=false;
    for(task = submission.child("task"); task; task = task.next_sibling("task"))
    {
        //read task id
        int readId;
        string readingValue=task.child("config").child_value("id");
        istringstream convertidTask(readingValue);
        convertidTask>>readId;
        if(readId==taskIdIn)
        {
            flagTaskFound=true;
            //cout<<"  read id="<<readId<<endl;
            break;
        }
    }


    if(!flagSubmissionFound || !flagTaskFound)
    {
        //cout<<" .Submission or task not found!"<<endl;
        return 0;
    }

    //read parameters of the task
    TheSubmission.readParameters(submission);
    TheTask.readParameters(task);



    return 1;
}



