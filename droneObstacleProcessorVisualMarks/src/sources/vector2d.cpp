// Vector2D.cpp: implementation of the Vector2D class.
//
//////////////////////////////////////////////////////////////////////

#include "vector2d.h"
#include "pose.h"
#include <math.h>
#include <assert.h>

Vector2D Vector2D::operator+=(const Vector2D& v2)
{
	x+=v2.x;
	y+=v2.y;
	return *this;
}
Vector2D Vector2D::operator/=(double n)
{
	x/=n;
	y/=n;
	return *this;
}
Vector2D Vector2D::operator/(double n)
{
	Vector2D ret;
	ret.x=x/n;
	ret.y=y/n;
	return ret;
}
Vector2D Vector2D::operator*(double n)
{
	Vector2D ret;
	ret.x=x*n;
	ret.y=y*n;
	return ret;
}
Vector2D Vector2D::operator*=(double n)
{
	x*=n;
	y*=n;
	return *this;
}
Vector2D Vector2D::operator+(const Vector2D& v2) const
{
	Vector2D ret;
	ret.x=x+v2.x;
	ret.y=y+v2.y;
	return ret;
}

Vector2D Vector2D::operator-(const Vector2D& v2) const
{
	Vector2D ret;
	ret.x=x-v2.x;
	ret.y=y-v2.y;
	return ret;
}

Vector2D::~Vector2D()
{

}
double Vector2D::operator*(const Vector2D& v2)
{
	return x*v2.x+y*v2.y;
}

double squaredDistance(const Vector2D& p1, const Vector2D& p2)
{
	return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
}

Vector2D midpoint(const Vector2D& p1, const Vector2D& p2)
{
	Vector2D ret;
	ret.x=(p1.x+p2.x)/2.0f;
	ret.y=(p1.y+p2.y)/2.0f;
	return ret;
}
	
Vector2D average(const vector<Vector2D>& v)
{
	Vector2D m;
	for(int i=0;i<v.size();i++)
	{
		m+=v[i];
	}
	m/=(double)v.size();
	return m;
}

