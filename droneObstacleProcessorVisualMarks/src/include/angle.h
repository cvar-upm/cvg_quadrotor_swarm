#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>


#ifndef RAD2DEG
#define RAD2DEG 180/M_PI
#endif
#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif

using namespace std;

class Angle
{
	friend ostream& operator<<(ostream& os, const Angle& a){os<<a.value;return os;} 
public:
	Angle(double ang = 0);
   virtual ~Angle(void);

   Angle operator -(const Angle& ang) const;
   Angle operator -() const;
   Angle operator +(const Angle& ang) const;
   void operator +=(const Angle& ang);
   bool operator < (const Angle& ang) const;
   bool operator > (const Angle& ang) const;
   bool operator <= (const Angle& ang) const;
   bool operator >= (const Angle& ang) const;

	void setValue(double v);
   inline double getValue() const {return value;};
   inline double getSine() const {updTrig();return sin_theta;}
   inline double getCosine() const {updTrig();return cos_theta;}

	Angle getOpposite();


protected:
	
   double value;
   double Normalize();

	inline void updTrig() const{if(!trig){cos_theta=cos(value);sin_theta=sin(value);trig=true;}}
	mutable double cos_theta;//for efficiency
	mutable double sin_theta;//for efficiency
	  mutable bool trig;
	  //mutable bool trig;

};
