#include "angle.h"


Angle::Angle(double ang)
{
    trig = false;
    value = ang;
    Normalize();
}

Angle::~Angle()
{

}

void Angle::setValue(double v)
{
	 trig = false;
    value = v;
    Normalize();
}

double Angle::Normalize()
{
    int n;
    if (value >= 0)
        n = (int) floor(value/M_PI);
    else
        n = (int) ceil(value/M_PI);

	value -= n*2*M_PI;

	return value;
}

Angle Angle::getOpposite()
{       
   double v = value + M_PI;
	Angle ret(v);
	return ret;
}
Angle Angle::operator -() const
{
	Angle ret(-value);
   return ret;
}
Angle Angle::operator-(const Angle& ang) const
{
   double ang_value = ang.value;
	double err=value-ang_value;
	if(fabs(err)>M_PI)
	{
		if(err>0)err-=(2*M_PI);
		else err+=(2*M_PI);
	}

	Angle ret(err);
   return ret;
}


Angle Angle::operator+(const Angle& ang) const
{
   double ang_value = ang.value;
	double angle = value + ang_value;
	Angle ret(angle);  //includes normalization
	return ret;


}
void Angle::operator +=(const Angle& ang) 
 { 
	*this=Angle(value + ang.value);
 }
bool Angle::operator<(const Angle& ang) const
{
	return (value < ang.value);

}

bool Angle::operator>(const Angle& ang) const
{
    return (value > ang.value);

}

bool Angle::operator<=(const Angle& ang) const
{
	return (value <= ang.value);

}

bool Angle::operator>=(const Angle& ang) const
{
    return (value >= ang.value);

}
