//////////////////////////////////////////////////////
//  droneTrajectoryPlanner.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef DRONE_TRAJECTORY_PLANNER_H
#define DRONE_TRAJECTORY_PLANNER_H


//I/O stream
//std::cout
#include <iostream>

//Vector
//std::vector
#include <vector>

//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream, std::ostringstream
#include <sstream>




//Map
#include "spaceMap.h"

//Trajectory planner
#include "trajectoryPlanner.h"


//Defines
//#define VERBOSE_DRONE_TRAJECTORY_PLANNER
#define SAVE_PRM_DRONE_TRAJECTORY_PLANNER


//Const
const double PRECISSION_IN_TRAJECTORY = 0.6; //m
const std::vector<double> EXTRA_RATIUS_ELLIPSE  = {0.0,0.0}; //m



namespace TrajectoryPlanner
{
    enum Result
    {
        NEW_TRAJECTORY_FOUND=0,
        NO_NEED_FOR_NEW_TRAJECTORY,
        UNABLE_TO_FIND_TRAJECTORY,
        TRAJECTORY_PARAMETERS_NOT_DEFINED,
        ERROR_FINDING_TRAJECTORY,
        ERROR_SETTING_POINT_INIT,
        ERROR_SETTING_POINT_FIN,
        ERROR_RECOVERING_TRAJECTORY,
        ERROR_NOTHING_DONE

    };
}


/////////////////////////////////////////
// Class DroneTrajectoryPlanner
//
//   Description
//
/////////////////////////////////////////
class DroneTrajectoryPlanner
{
    //Trajectory planner
protected:
    RobotTrajectoryPlanner* MyRobotTrajectoryPlanner;

    //settings
protected:
    bool flagLogging;

    //Id Drone
protected:
    unsigned int idDrone;

    //Files
protected:
    std::string configFile;
    std::vector<std::string> prmFiles;

    //Logs
protected:
    std::vector<std::string> logFiles;


    //Const & destr
public:
    DroneTrajectoryPlanner();
    ~DroneTrajectoryPlanner();

    //Init
protected:
    bool init();
    //Clear
protected:
    bool clear();

    //Configure
public:
    int configure(bool setLogs);

    //Open
public:
    int open(int idDroneIn, std::string configFileIn, std::vector<std::string> prmFilesIn);
    int open(int idDroneIn, std::string configFileIn, std::vector<std::string> prmFilesIn, std::vector<std::string> logFilesIn);
    //Close
public:
    int close();


    //Reset
public:
    bool reset();
    //Start
public:
    bool start();
    //Stop
public:
    bool stop();

    //Run
public:
    //bool run(bool& trajectoryFound, bool flagReplanifyAlways=false);
    bool run(TrajectoryPlanner::Result& resultOut, bool flagReplanifyAlways=false);


    //Point init
protected:
    std::vector<double> pointInit;
    bool newPointInit;
public:
    int setPointInit(std::vector<double> pointInitIn);


    //Point fin
protected:
    bool newPointFin;
    std::vector<double> pointFin;   
public:
    int setPointFin(std::vector<double> pointFinIn);


    //Obstacles
protected:
    bool newObstacles;
    std::vector<unsigned int> dynamicObstaclesAdded; //id of the obstacles


    //Society pose. JL Fix??? Why society?? They are obstacles!!
protected:
    bool newSocietyPose;
    std::vector<unsigned int> societyIdsAdded; //Id of other drones


    //flag find trajectory
protected:
    bool flagFindTrajectory;


    //Trajectory
protected:
    std::vector< std::vector<double> > trajectory;
public:
    int getTrajectory(std::vector< std::vector<double> > &trajectoryOut);


};


/////////////////////////////////////////
// Class DroneTrajectoryPlanner2d
//
//   Description
//
/////////////////////////////////////////
class DroneTrajectoryPlanner2d : public DroneTrajectoryPlanner
{
public:
    DroneTrajectoryPlanner2d();
    ~DroneTrajectoryPlanner2d();

    //Obstacles
public:
    //int setObstacles(std::vector<EllipseObstacle2d> EllipsesIn, unsigned int minId, unsigned int maxId, bool keepOldObstacles=false);
    //int setObstacles(std::vector<RectangleObstacle2d> RectanglesIn, unsigned int minId, unsigned int maxId, bool keepOldObstacles=false);
    int setObstacles(std::vector<EllipseObstacle2d> EllipsesIn, std::vector<RectangleObstacle2d> RectanglesIn);


    //Society pose
public:
    int setSocietyPose(std::vector<Robot2d> societyIn);


};


/////////////////////////////////////////
// Class DroneTrajectoryPlanner3d
//
//   Description TODO PABLO!!!
//
/////////////////////////////////////////
class DroneTrajectoryPlanner3d : public DroneTrajectoryPlanner
{

};




#endif
