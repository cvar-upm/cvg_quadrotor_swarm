//////////////////////////////////////////////////////
//  droneObstacleProcessor.cpp
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente
//
//  Last modification on: Oct 29, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneObstacleProcessor.h"



using namespace std;




DroneObstacleProcessor::DroneObstacleProcessor()
{
    if(!init())
    {
        cout<<"[DOP] Error init"<<endl;
    }

    return;
}



DroneObstacleProcessor::~DroneObstacleProcessor()
{
	close();
	return;
}

bool DroneObstacleProcessor::init()
{
    landmarks.clear();
    obstacles.clear();

    return true;
}


int DroneObstacleProcessor::open(std::string fileParamsLocalization)
{
    //Init obstacle_processor
    obstacle_processor.initialization(fileParamsLocalization);

	
	//End
    return 1;
}


int DroneObstacleProcessor::close()
{
    landmarks.clear();
    obstacles.clear();

    return 1;
}


bool DroneObstacleProcessor::reset()
{
    landmarks.clear();
    obstacles.clear();

    return true;
}



bool DroneObstacleProcessor::start()
{


    //End
    return 1;
}


bool DroneObstacleProcessor::stop()
{
    return 1;
}


bool DroneObstacleProcessor::run()
{

    /////////////////////////////////////////////////////////
    //calculate obstacles
    obstacle_processor.getObstacles(landmarks);

    //Set obstacles in obstacles
    obstacles.clear();
    for(unsigned int i=0;i<obstacle_processor.detectedObstacles.size();i++)
    {
        obstacles.push_back(obstacle_processor.detectedObstacles[i]);
    }



    return false;
}


int DroneObstacleProcessor::setLandmarks(std::vector<Landmark3D> landmarksIn)
{
    landmarks=landmarksIn;
    return 1;
}


int DroneObstacleProcessor::getObstacles(std::vector<Obstacle2D*>& obstaclesOut)
{
    obstaclesOut=obstacles;
    return 1;
}
