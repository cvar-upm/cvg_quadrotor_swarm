#pragma once

#include <vector>
#include <cmath>

using namespace std;

class Landmark3D  
{
public:

    Landmark3D(void){}
	Landmark3D(int num,double ix=0,double iy=0, double iz=0, double iroll=0, double ipitch=0, double iyaw=0){id=num;x=ix;y=iy;z=iz;roll=iroll;pitch=ipitch;yaw=iyaw;is_known = false;}
	
    virtual ~Landmark3D(){}
	
	int id;
	
	bool is_known;
	
    inline bool isKnown(){return is_known;}
	
	
	double x;
	double y;
	double z;
	
	double roll;
	double pitch;
	double yaw;
	
	
};


