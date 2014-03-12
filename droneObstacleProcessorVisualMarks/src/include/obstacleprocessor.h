#pragma once

#include <fstream>
#include <string>
#include <sstream>

#include <iostream>
#include <stdio.h>

#include "obstacle.h"
#include "landmark3d.h"
#include <vector> 
#include <map>


#include "circle.h"

using namespace std;

struct comp_operator
{
  bool operator()(Landmark3D lm1, Landmark3D lm2) const
  {
	 return (lm1.id > lm2.id);   // so that the set is ordered in descending order
  }
};
typedef map<int,Landmark3D,comp_operator> Landmark3DMap;


class ObstacleProcessor
{

public:
	ObstacleProcessor(void);
	~ObstacleProcessor(void);
	
    vector<Obstacle2D*> getObstacles(vector<Landmark3D> v);
	
    vector<Obstacle2D*> detectedObstacles;

    std::vector<Circle> existingPoles;
	
    void initialization(std::string paramsFileName);
	
	double wall_width;
	double wall_length;
	double large_window_length;
	double small_window_length;
	double known_pole_rad;
	double unknown_pole_rad;

    double aruco_width;
	
	int num_landmarks_wind;
	int windows_distribution_option;
	
};


