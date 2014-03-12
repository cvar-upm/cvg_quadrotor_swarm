//////////////////////////////////////////////////////
//  spaceMap.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef SPACE_MAP_H
#define SPACE_MAP_H



//C Math
//sin(), cos(), pow(), abs()
#include <cmath>

//I/O Steam
//std::cout
#include <iostream>

//Vector
//std::vector
#include <vector>



//#define VERBOSE_SPACE_MAP


/////////////////////////////////////////
// Class ObstacleInMap
//
//   Description
//
/////////////////////////////////////////
class ObstacleInMap
{
    //Obstacle Id
protected:
    unsigned int obstacleId;

    //Type of obstacle
protected:
    int obstacleType;

    //Variances definition
public:
    std::vector<double> variancesObstacle;

public:
    ObstacleInMap();
    virtual ~ObstacleInMap();

    int init();
    int clear();

public:
    int setObstacleId(unsigned int obstacleIdIn);
    unsigned int getObstacleId();

public:
    int setObstacleType(int obstacleTypeIn);
    int getObstacleType();

    virtual double implicitEquation(std::vector<double> pointIn, std::vector<double> robotDimensions);

};



/////////////////////////////////////////
// Class EllipseObstacle2d
//
//   Description
//
/////////////////////////////////////////
class EllipseObstacle2d : public ObstacleInMap
{
    //Object definition
protected:
    std::vector<double> centerPoint;
    std::vector<double> radius;
    double yawAngle;


public:
    EllipseObstacle2d();
    ~EllipseObstacle2d();

    int init();
    int clear();

public:
    int getParameters(unsigned int& idOut, std::vector<double>& centerPointOut, std::vector<double>& radiusOut, double& yawAngleOut);
    int setParameters(unsigned int idIn, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn); //equivalent to define

public:

    int define(unsigned int id, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn);

    double implicitEquation(std::vector<double> pointIn, std::vector<double> robotDimensions);

};


/////////////////////////////////////////
// Class RectangleObstacle2d
//
//   Description
//
/////////////////////////////////////////
class RectangleObstacle2d : public ObstacleInMap
{
    //Object definition
protected:
    std::vector<double> centerPoint;
    std::vector<double> size;
    double yawAngle;


public:
    RectangleObstacle2d();
    ~RectangleObstacle2d();

    int init();
    int clear();

    //TODO JL
public:
    int getParameters(unsigned int& idOut, std::vector<double>& centerPointOut, std::vector<double>& sizeOut, double& yawAngleOut);
    int setParameters(unsigned int idIn, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn); //equivalent to define


public:

    int define(unsigned int id, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn);

    double implicitEquation(std::vector<double> pointIn, std::vector<double> robotDimensions);


};


/////////////////////////////////////////
// Class EllipsoidObstacle3d
//
//   Description TODO PABLO!!
//
/////////////////////////////////////////
class EllipsoidObstacle3d : public ObstacleInMap
{

};






/////////////////////////////////////////
// Class WorldMap
//
//   Description
//
/////////////////////////////////////////
class WorldMap
{
    //Size
protected:
    std::vector<double> dimension;
    std::vector<double> initPoint;


public:
    int setDimensions(std::vector<double> dimensionIn);
    int getDimensions(std::vector<double> &dimensionOut);

public:
    int getDimension(unsigned int &dimensionNum);

public:
    int setInitPoint(std::vector<double> initPointIn);
    int getInitPoint(std::vector<double> &initPointOut);


protected:
    //Obstacles
    // See: http://stackoverflow.com/questions/5200663/virtual-functions-iterating-over-a-vectorbase-class-that-is-populated-with-su
    //std::vector< std::unique_ptr<ObstacleInMap> > obstaclesList;
    std::vector<ObstacleInMap*> obstaclesList;

public:
    int getNumberObstacles();

protected:
    int getHandleObstacle(unsigned int numObstacleI, ObstacleInMap **obstacleHandle);

public:
    WorldMap();
    ~WorldMap();

    int init();
    int clear();

protected:
    int findObstacle(unsigned int &numObstacleI, unsigned int idObstacle);

public:
    bool existObstacle(unsigned int idObstacle);

public:
    //double implicitEquationObstacleI(int numObstacleI, std::vector<double> pointIn, std::vector<double> robotDimensions);
    int implicitEquationObstacleI(double &implicitEqValOut, unsigned int numObstacleI, std::vector<double> pointIn, std::vector<double> robotDimensions);

    int deleteObstacle(unsigned int idObstacle);

    //For debugging
public:
    int listIdObstacles();

};



/////////////////////////////////////////
// Class WorldMap2d
//
//   Description
//
/////////////////////////////////////////
class WorldMap2d : public WorldMap
{
public:
    WorldMap2d();
    ~WorldMap2d();

    int init();
    int clear();

    //Ellipse obstacle 2d
protected:
    int getHandleEllipseObstacle2d(unsigned int numObstacleI, EllipseObstacle2d **obstacleHandle);

public:
    int addEllipseObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn);
    int updateEllipseObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn);

    //Rectangle obstacle 2d
public:
    int addRectangleObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn);
    int updateRectangleObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn);

    //For debugging
public:
    int listObstacles();

};


/////////////////////////////////////////
// Class WorldMap3d
//
//   Description TODO PABLO!!
//
/////////////////////////////////////////
class WorldMap3d: public WorldMap
{

};





#endif
