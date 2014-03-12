//////////////////////////////////////////////////////
//  trajectoryPlanner.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H




//C Math
//sqrt(), pow()
#include <cmath>

//I/O Stream
//cout
#include <iostream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>

//String
//std::string, std::getline()
#include <string>

//Vector
//std::vector
#include <vector>


//Time
#include <ctime>

//Uniform random generation
#include <random>



////XML parser
#include "pugixml.hpp"


//Map
#include "spaceMap.h"

//PRM
#include "probabilisticRoadMap.h"

//Potencial field map
#include "potencialFieldMap.h"

//Discrete search
#include "discreteSearch.h"




//#define VERBOSE_ROBOT_TRAJECTORY_PLANNER



///// Constants
const double DISTANCE_NEAR_ROBOT=5.0; //6.0

const unsigned int NUMBER_INIT_ROBOT_ID=0;
const double TOL_FINDING_DISTANCE_POINT_TO_RECT=0.15;




/////////////////////////////////////////
// Class Robot
//
//   Description
//
/////////////////////////////////////////
class Robot
{
protected:
    unsigned int id;

    std::vector<double> centerPoint;
    std::vector<double> sizeDimensions;


public:
    Robot();
    ~Robot();

    unsigned int getRobotId();

};


/////////////////////////////////////////
// Class Robot2d
//
//   Description
//
/////////////////////////////////////////
class Robot2d : public Robot
{
protected:
    double yawAngle;

public:
    Robot2d();

    int getRobotParameters(unsigned int& idOut, std::vector<double>& centerPointOut, std::vector<double>& sizeDimensionsOut, double& yawAngleOut);
    int setRobotParameters(unsigned int idIn, std::vector<double> centerPointIn, std::vector<double> sizeDimensionsIn, double yawAngleIn);

};


/////////////////////////////////////////
// Class Robot3d
//
//   Description TODO PABLO!!
//
/////////////////////////////////////////
class Robot3d : public Robot
{
protected:
    std::vector<double> eulerAngles;



};





/////////////////////////////////////////
// Class RobotTrajectoryPlanner
//
//   Description
//
/////////////////////////////////////////
class RobotTrajectoryPlanner
{
protected:
    ProbabilisticRoadMap MyPRM;
    AStarWithPotencialFieldMap MyAStar;

protected:
    WorldMap* MyMap;
    PotencialFieldMap* MyPotencialFieldMap;

protected:
    std::vector<Robot*> othersRobotPose; //Others robots

protected:
    std::vector<double> pointInit;
    std::vector<double> pointFin;

protected:
    std::vector< std::vector<double> > trajectoryFound;

protected:
    std::vector<double> robotDimensions;

protected:
    double maxCostToReplan;

public:
    RobotTrajectoryPlanner();
    virtual ~RobotTrajectoryPlanner();

public:
    int init();
    int clear();

protected:
    //MAP
    int initMap(std::vector<double> dimensionMyMapIn, std::vector<double> initPointMyMapIn);

    //PRM
    int initPRM(std::string prmConfigFile, std::string nodesListFile, std::string nodesRelationFile);
    int initPRM(std::string nodesListFile, std::string nodesRelationFile);

    //Potencial field map
    virtual int initPotFieldMap(std::vector<double> robotDimensions);

    //A*
    int initAStar();

public:
    //Init plannifier
    int initComponents(std::string configFile, std::string PRMNodesListFile, std::string PRMNodesRelationFile);

public:
    //Map
    virtual int createMap(std::string mapConfigFile);

    //Points init and fin
public:
    int setPoints(std::vector<double> pointInitIn, std::vector<double> pointFinIn);
public:
    int setPointInit(std::vector<double> pointInitIn);
    int setPointFin(std::vector<double> pointFinIn);


    ///Dynamic Obstacle functions
public:
    //Delete
    int deleteObstacle(unsigned int idObstacle);

    int clearQuadRotors();


    //Trajectory
public:
    //Check full trajectory
    virtual int checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn);//JL to be finished. Robots near!!

    //check trajectory from point
    virtual int checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn, std::vector<double> robotPointIn);//JL to be finished. Robots near!!

public:
    virtual int findTrajectory(bool& isTrajectoryFound);

public:
    int getTrajectory(std::vector< std::vector<double> > &trajectoryFoundOut);

    //Save results to view in Matlab!
public:
    int savePRM(std::string nodesList, std::string nodesRelation);
    int savePlannerResult(std::string solutionNodes, std::string simplifSolNodes);
    int saveResult(std::string nodesList, std::string nodesRelation, std::string solutionNodes, std::string simplifSolNodes);



public:
    //JL to fix!!!
    virtual int distancePointToRect(double &distance, std::vector<double> &pointProjected, std::vector<double> pointSearch, std::vector<double> point1, std::vector<double> point2);

};


/////////////////////////////////////////
// Class RobotTrajectoryPlanner2d
//
//   Description
//
/////////////////////////////////////////
class RobotTrajectoryPlanner2d : public RobotTrajectoryPlanner
{
public:
    RobotTrajectoryPlanner2d();
    ~RobotTrajectoryPlanner2d();

public:
    int init();
    int clear();


protected:
    //Potencial field map
    int initPotFieldMap(std::vector<double> robotDimensions);

public:
    //Map
    int createMap(std::string mapConfigFile);

    //Trajectory
public:
    int findTrajectory(bool& isTrajectoryFound);


    ///Dynamic Obstacle functions
public:
    //Ellipse
    int setEllipse(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn);
    //Rectangle
    int setRectangle(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn);
    //Other QR
    int setQuadRotor(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> qrSizeIn, double yawAngleIn);


    //Trajectory
public:
    //Check full trajectory
    int checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn);//JL to be finished. Robots near!!
    //check trajectory from point
    int checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn, std::vector<double> robotPointIn);//JL to be finished. Robots near!!


public:
    //JL to fix!!!
    int distancePointToRect(double &distance, std::vector<double> &pointProjected, std::vector<double> pointSearch, std::vector<double> point1, std::vector<double> point2);

};


/////////////////////////////////////////
// Class RobotTrajectoryPlanner3d
//
//   Description TODO PABLO!!!
//
/////////////////////////////////////////
class RobotTrajectoryPlanner3d : public RobotTrajectoryPlanner
{

};



#endif
