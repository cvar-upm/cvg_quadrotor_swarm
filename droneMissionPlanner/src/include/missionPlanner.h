/*
 * 
 *
 *  Created on: 
 *      Author: Jose Luis Sanchez-Lopez
 */

#ifndef MISSION_PLANNER_H
#define MISSION_PLANNER_H




//Math
#include <math.h>


#include <cstdlib>
#include <ctime>



#include <iostream>


#include <string>
#include <sstream>
#include <vector>

#include <algorithm>    // std::sort

//ifstream
#include <fstream>



////XML parser
#include "pugixml.hpp"




#define CMD_TAKEOFF             "takeOff"
#define CMD_HOVER               "hover"
#define CMD_LAND                "land"
#define CMD_TRAJ_MOVEMENT       "move"
#define CMD_SLEEP               "sleep"






namespace missionPlanner
{
    //tasks types
    enum taskTypes
    {
        takeOff, //take off
        hover, //hover
        land, //land
        sleep, //wait there
        trajectoryMovement, //move in trajectory mode
        positionMovement, //move in position mode

        unknown
    };


}



class Submission
{
public:
    int id;
    std::vector<double> precission;
    bool loop;
    double duration;
    double battery;


public:
    int readParameters(const pugi::xml_node &submission);

};


class Task : public Submission
{
public:
    //Task Type
    missionPlanner::taskTypes type;

    //Point to go
    std::vector<double> point;
    std::vector<bool> pointLoaded;

    //Yaw angle
    double yaw;
    bool yawLoaded;
    //point to look
    std::vector<double> pointToLook;
    std::vector<bool> pointToLookLoaded;


public:
    int readParameters(const pugi::xml_node &task);

};



class RobotMissionPlanner
{


private:
    pugi::xml_document doc;


public:
    RobotMissionPlanner();
    ~RobotMissionPlanner();


public:
    int init();
    int clear();


public:
    std::vector<int> submissionTree; //tree with the id of the submissions
    std::vector< std::vector<int> > taskTree; //tree with the id of the tasks of each submission

public:
    int generateMissionTree();
    int setMission(std::string missionConfigFile);



    //general
public:
    std::vector<double> defaultPrecission;
    double yawPrecission;



    //Submission
public:
    Submission TheSubmission;


    //task
public:
    Task TheTask;






public:
    int readTask(int submissionIdIn, int taskIdIn);





};







#endif
