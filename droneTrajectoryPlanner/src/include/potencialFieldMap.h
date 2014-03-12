//////////////////////////////////////////////////////
//  potencialFieldMap.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef POTENCIAL_FIELD_MAP_H
#define POTENCIAL_FIELD_MAP_H



//C Math
//exp(), pow(), sqrt(), atan2(), floor()
#include <cmath>

//I/O Stream
//cout
#include <iostream>

//Vector
//std::vector
#include <vector>



//spaceMap
#include "spaceMap.h"


//Const values
const double OBSTACLE_ALTITUDE_DEF=1e10;
const double OBSTACLE_PRECISSION_DEF=20.0;//10.0;
const double STEP_SIZE_IN_INTEGRATION_S=0.1;


//#define VERBOSE_POTENCIAL_FIELD_MAP



/////////////////////////////////////////
// Class ObstacleInPotencialMap
//
//   Description
//
/////////////////////////////////////////
class ObstacleInPotencialMap
{

protected:
    //Definition of potential map
    double obstacleAltitudeDef;
    double obstaclePrecissionDef;


public:
    ObstacleInPotencialMap();
    ~ObstacleInPotencialMap();

    int init();
    int clear();


public:
    int define(double obstacleAltitudeDefIn, double obstaclePrecissionDefIn);


public:
    int calculateAltitudePotMap(double &valueAltitudeOut, double valueImplicitEquation);



};



/////////////////////////////////////////
// Class PotencialFieldMap
//
//   Description
//
/////////////////////////////////////////
class PotencialFieldMap
{
protected:
    unsigned int dimension;

public:
    int setDimension(unsigned int dimensionIn);

protected:
    //Vector obstacles
    std::vector<ObstacleInPotencialMap> ObstaclesList;


protected:
    //Puntos inicial y final
    std::vector<double> pointInit;
    std::vector<double> pointFin;


    //Definition of potential map
    double pointInitAltitude;
    double pointFinAltitude;


protected:
//public:
    //Real map
    WorldMap* RealMap;



protected:
    //Robot definition
    std::vector<double> robotDimensions;


public:

    PotencialFieldMap();
    virtual ~PotencialFieldMap();

    int init();
    int clear();


public:

    int setRobotDimensions(std::vector<double> robotDimensionsIn);

    virtual int setRealMap(WorldMap *RealMapIn);


    int setPointsInitFin(std::vector<double> pointInitIn, std::vector<double> pointFinIn, double pointInitAltitudeIn=-1.0, double pointFinAltitudeIn=0.0);


    int setPointInit(std::vector<double> pointInitIn, double pointInitAltitudeIn=-1.0);

    int setPointFin(std::vector<double> pointFinIn, double pointFinAltitudeIn=0.0);




    //Potencial map pointInit - pointFin curve
protected:
    std::vector<double> potFieldMapCurve;

    virtual int calculatePotMapCurve();

public:
    int getPotMapCurve(std::vector<double> &potFieldMapCurve);


public:

    int updateObstacleMap();


public:

    virtual int calculateAltitudePotMapCurve(double &valueAltitudeOut, std::vector<double> pointIn);

    int calculateAltitudePotMapObstacleI(double &valueAltitudeOut, std::vector<double> pointIn, unsigned int numObstacleI);




    virtual int calculateDistPotMap(double &distanceOut, std::vector<double> pointIn, std::vector<double> pointFin, bool useObstacles=true, bool useCurve=true);

};


/////////////////////////////////////////
// Class PotencialFieldMap2d
//
//   Description
//
/////////////////////////////////////////
class PotencialFieldMap2d : public PotencialFieldMap
{


protected:
    int calculatePotMapCurve();


public:
    PotencialFieldMap2d();
    ~PotencialFieldMap2d();

    int init();
    int clear();


public:
    int setRealMap(WorldMap2d *RealMapIn);


public:
    int calculateAltitudePotMapCurve(double &valueAltitudeOut, std::vector<double> pointIn);


public:
    int calculateDistPotMap(double &distanceOut, std::vector<double> pointIn, std::vector<double> pointFin, bool useObstacles=true, bool useCurve=true);


};



/////////////////////////////////////////
// Class PotencialFieldMap3d
//
//   Description: TODO PABLO!!
//
/////////////////////////////////////////
class PotencialFieldMap3d : public PotencialFieldMap
{

};



#endif
