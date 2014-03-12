#pragma once

#include <vector>
#include <math.h>

#include "pose.h"

#include "obstacle.h"

using namespace std;

class Rectangle  : public Obstacle2D
{


public:
	Rectangle(Pose p, double x_length=1, double y_length=1){pose=p;xl=x_length;yl=y_length; type = RECTANGLE;}
	virtual ~Rectangle(){}

	Pose pose;
	double xl;
	double yl;
};




