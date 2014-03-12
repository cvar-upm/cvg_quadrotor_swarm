#pragma once

#include <vector>
#include <cmath>



class Observation3D
{
public:
	Observation3D(int num=-1,double ix=0,double iy=0, double iz=0, double iroll=0, double ipitch=0, double iyaw=0){x=ix;y=iy;z=iz;roll=iroll;pitch=ipitch;yaw=iyaw;id=num;is_known = false;}
	virtual ~Observation3D(){};
	
	double x;
	double y;
	double z;
	
	double roll;
	double pitch;
	double yaw;
	
	int id;
	
	bool is_known;
	inline bool isKnown(){return is_known;};
	
};


