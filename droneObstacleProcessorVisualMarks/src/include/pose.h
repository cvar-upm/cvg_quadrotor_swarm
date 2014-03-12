#pragma once

#include <iostream>
#include <math.h>

#include "vector2d.h"
#include "angle.h"

using namespace std;

class Vector2D;
//mathematics utilities related to poses


class Pose 
{
	friend ostream& operator<<(ostream& os, const Pose& p);
	friend istream& operator>>(istream& os, Pose& p);

public:
//constructors
	Pose(double ix=0,double iy=0, Angle itheta=0){x=ix;y=iy;theta=itheta;}
	virtual ~Pose(void){;}

	double Module(double L=2) const;//returns sqrt(x2+y2+L2theta2)

//a (+) b
	Pose operator*(const Pose& p) const;
	void operator*=(const Pose& p);

	Pose operator-(const Pose& p)const;///< normal substraction The - takes into account angle substraction -PI, PI
	Pose operator+(const Pose& p)const;///< normal addition
	void operator+=(const Pose& p);///< normal addition
	Vector2D operator*(const Vector2D& v) const;	//<composition of the pose and Vector2D v
	Pose operator -() const;

//(-)a(+)b
	Pose relative(const Pose& b) const;

//(-)a(+)b
	Vector2D relative(const Vector2D& v) const;

	double x;
	double y;
	Angle theta;

	inline void setValues(double ix=0,double iy=0, Angle itheta=Angle(0)){x=ix;y=iy;theta=itheta;}
	inline void setValues(const Pose& p){x=p.x;y=p.y;theta=p.theta;}



};



