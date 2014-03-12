#pragma once

#include <vector>
#include <math.h>

using namespace std;

class Vector2D 
{
public:
	Vector2D(double ix=0,double iy=0){x=ix;y=iy;}
	virtual ~Vector2D();
	
	double operator*(const Vector2D& v2);
	Vector2D operator-(const Vector2D& v2) const;
	Vector2D operator-() const{return Vector2D(-x,-y);}
	Vector2D operator+(const Vector2D& v2) const;
	Vector2D operator+=(const Vector2D& v2);
	Vector2D operator/=(double n);
	Vector2D operator/(double n);
	Vector2D operator*=(double n);
	Vector2D operator*(double n);
	double module()const{return sqrt(x*x+y*y);}
	double argument()const{return atan2(y,x);}

	double x;
	double y;
};


double squaredDistance(const Vector2D& p1, const Vector2D& p2);
Vector2D midpoint(const Vector2D& p1, const Vector2D& p2);
Vector2D average(const vector<Vector2D>& v);


