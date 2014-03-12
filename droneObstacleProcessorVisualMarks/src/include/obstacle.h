#pragma once

#include <set>

enum obstacleType
{CIRCLE,RECTANGLE};

class Obstacle2D
{

public:
	Obstacle2D(int i=-1){id=i;type=-1;}
	virtual ~Obstacle2D(){}
	
	inline int getType(){return type;}

	//virtual void applyTransform(const Pose& transf)=0;//<changes the reference frame of the feature feature=newFrame(+)feature;

	inline int getID(){return id;};

	int id;//unique ID

    std::set<int> arucoIds;

	int type;
	
	int weight;
	

};


