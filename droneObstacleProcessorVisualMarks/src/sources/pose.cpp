#include "pose.h"
#include <assert.h>

ostream& operator<<(ostream& os, const Pose& p)
{
    double theta = p.theta.getValue();
	os<<p.x<<" "<<p.y<<" "<<theta;
	return os;
}
istream& operator>>(istream& os, Pose& p)
{
    double theta;
	os>>p.x>>p.y>>theta;
	p.theta=Angle(theta);

	return os;
}


double Pose::Module(double L) const
{
	return sqrt(x*x+y*y+L*L*theta.getValue()*theta.getValue());
}
Pose Pose::operator-(const Pose& p)const
{
	Pose r;
	r.x=x-p.x;
	r.y=y-p.y;
	r.theta=theta-p.theta;
	return r;
}
Pose Pose::operator+(const Pose& p)const
{
	Pose r;
	r.x=x-p.x;
	r.y=y-p.y;
	r.theta=theta+p.theta;
	return r;
}
void Pose::operator+=(const Pose& p)
{
//	theta.trig = false;
	x=x+p.x;
	y=y+p.y;
	theta=theta+p.theta;
   // cout << "trig " << theta.trig << endl;
}
Pose Pose::operator -() const
{
	double cos_theta = theta.getCosine();
	double sin_theta = theta.getSine();
	Pose pos;
	pos.x= -x*cos_theta-y*sin_theta;
	pos.y= x*sin_theta-y*cos_theta;
	pos.theta= Angle()-theta;
	return pos;
}


Pose Pose::operator*(const Pose& p)const
{
    double cos_theta = theta.getCosine();
	double sin_theta = theta.getSine();
	Pose pos;
	pos.x=x+p.x*cos_theta-p.y*sin_theta;
	pos.y=y+p.x*sin_theta+p.y*cos_theta;
	pos.theta=theta+p.theta;
	return pos;
}

void Pose::operator*=(const Pose& p)
{
//	theta.trig = false;
	*this=(*this)*p;
}
Vector2D Pose::operator*(const Vector2D& p)const
{
	//updTrig();
    double cos_theta = theta.getCosine();
	double sin_theta = theta.getSine();
	Vector2D pos;
	pos.x=x+p.x*cos_theta-p.y*sin_theta;
	pos.y=y+p.x*sin_theta+p.y*cos_theta;
	return pos;
}

//(-)a(+)b
Pose Pose::relative(const Pose& b) const
{
	//updTrig();
    double cos_theta = theta.getCosine();
	double sin_theta = theta.getSine();
	Pose r;
	r.x=-x*cos_theta-y*sin_theta+b.x*cos_theta+b.y*sin_theta;
	r.y=x*sin_theta-y*cos_theta-b.x*sin_theta+b.y*cos_theta;
	r.theta=b.theta-theta;
	return r;
}


//(-)a(+)b
Vector2D Pose::relative(const Vector2D& b) const
{
//	updTrig();
	double cos_theta = theta.getCosine();
	double sin_theta = theta.getSine();
	Vector2D r;
	r.x=-x*cos_theta-y*sin_theta+b.x*cos_theta+b.y*sin_theta;
	r.y=x*sin_theta-y*cos_theta-b.x*sin_theta+b.y*cos_theta;
	return r;
}

