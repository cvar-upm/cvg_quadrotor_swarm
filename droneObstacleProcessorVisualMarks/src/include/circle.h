#pragma once

#include <vector>
#include <math.h>
//#include "drlgl/globject.h"

#include "vector2d.h"

#include "obstacle.h"

using namespace std;

class Circle  : public Obstacle2D
{
public:
	Circle(double ix=0,double iy=0,double rad=0.4){center.x=ix;center.y=iy;radius=rad;type = CIRCLE;}
	virtual ~Circle(){}

	Vector2D center;
	double radius;
	
};




