//////////////////////////////////////////////////////
//  droneObstacleProcessor.h
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente
//
//  Last modification on: Oct 29, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef DRONE_OBSTACLE_PROCESSOR_H
#define DRONE_OBSTACLE_PROCESSOR_H





//Math
#include <cmath>


#include <cstdlib>
#include <ctime>



#include <iostream>

#include <fstream>

#include <vector>

#include <sstream>

#include "obstacleprocessor.h"
#include "obstacle.h"
#include "rectangle.h"
#include "circle.h"


/////////////////////////////////////////
// Class DroneObstacleProcessor
//
//   Description
//
/////////////////////////////////////////
class DroneObstacleProcessor
{	

protected:
     ObstacleProcessor obstacle_processor;
     std::vector<Obstacle2D*> obstacles;
     std::vector<Landmark3D> landmarks;

    //Constructor and destructor
public:
    DroneObstacleProcessor();
    ~DroneObstacleProcessor();
	
    //open and close
public:
    int open(std::string fileParamsLocalization);
    int close();
	
    //init
public:
    bool init();

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
    bool run();

    //managing landmarks and obstacles
public:
     int setLandmarks(std::vector<Landmark3D> landmarksIn);
     int getObstacles(std::vector<Obstacle2D*>& obstaclesOut);

};




#endif
