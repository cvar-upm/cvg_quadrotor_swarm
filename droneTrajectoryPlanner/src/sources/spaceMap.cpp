//////////////////////////////////////////////////////
//  spaceMap.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "spaceMap.h"


using namespace std;



////////////////////// ObstacleInMap /////////////////////

ObstacleInMap::ObstacleInMap()
{
    init();
    return;
}

ObstacleInMap::~ObstacleInMap()
{
    clear();
    return;
}



double ObstacleInMap::implicitEquation(std::vector<double> pointIn, std::vector<double> robotDimensions)
{
    //cout<<"General implicit eq"<<endl;
    return 0.0;
}






int ObstacleInMap::init()
{
    obstacleId=-1;

    return 1;
}

int ObstacleInMap::clear()
{
    variancesObstacle.clear();

    return 1;
}


int ObstacleInMap::setObstacleId(unsigned int obstacleIdIn)
{
    obstacleId=obstacleIdIn;
    return 1;
}

unsigned int ObstacleInMap::getObstacleId()
{
    return obstacleId;
}


int ObstacleInMap::setObstacleType(int obstacleTypeIn)
{
    obstacleType=obstacleTypeIn;
    return 1;
}

int ObstacleInMap::getObstacleType()
{
    return obstacleType;
}






////////////////////// EllipseObstacle2d /////////////////////


EllipseObstacle2d::EllipseObstacle2d()
{
    init();
    return;
}


EllipseObstacle2d::~EllipseObstacle2d()
{
    clear();
    return;
}


int EllipseObstacle2d::init()
{
    centerPoint.resize(2);
    radius.resize(2);
    variancesObstacle.resize(2);

    return 1;
}

int EllipseObstacle2d::clear()
{
    ObstacleInMap::clear();

    centerPoint.clear();
    radius.clear();
    //robotDimensions.clear();
    variancesObstacle.clear();

    return 1;
}


int EllipseObstacle2d::getParameters(unsigned int& idOut, std::vector<double>& centerPointOut, std::vector<double>& radiusOut, double& yawAngleOut)
{
    idOut=obstacleId;
    centerPointOut=centerPoint;
    radiusOut=radius;
    yawAngleOut=yawAngle;
    return 1;
}


int EllipseObstacle2d::setParameters(unsigned int idIn, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn)
{
    return define(idIn,centerPointIn,radiusIn,yawAngleIn);
}


int EllipseObstacle2d::define(unsigned int id, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn)
{
    setObstacleType(1);

    obstacleId=id;
    centerPoint=centerPointIn;
    radius=radiusIn;
    yawAngle=yawAngleIn;

    return 1;
}


double EllipseObstacle2d::implicitEquation(std::vector<double> pointIn, std::vector<double> robotDimensions)
{
    //cout<<"Ellipse implicit eq"<<endl;

    std::vector<double> pointRotated(2);

    pointRotated[0]=(pointIn[0]-centerPoint[0])*cos(yawAngle)        +(pointIn[1]-centerPoint[1])*sin(yawAngle);
    pointRotated[1]=(pointIn[0]-centerPoint[0])*(-1.0)*sin(yawAngle) +(pointIn[1]-centerPoint[1])*cos(yawAngle);


    double value=0.0;


    value=pow(pointRotated[0]/(radius[0]+robotDimensions[0]/2.0),2)+pow(pointRotated[1]/(radius[1]+robotDimensions[1]/2.0),2)-1;


    return value;
}





////////////////////// RectangleObstacle2d /////////////////////


RectangleObstacle2d::RectangleObstacle2d()
{
    init();
    return;
}


RectangleObstacle2d::~RectangleObstacle2d()
{
    clear();
    return;
}


int RectangleObstacle2d::init()
{
    centerPoint.resize(2);
    size.resize(2);
    variancesObstacle.resize(2);

    return 1;
}

int RectangleObstacle2d::clear()
{
    ObstacleInMap::clear();

    centerPoint.clear();
    size.clear();
    //robotDimensions.clear();
    variancesObstacle.clear();

    return 1;
}


int RectangleObstacle2d::getParameters(unsigned int& idOut, std::vector<double>& centerPointOut, std::vector<double>& sizeOut, double& yawAngleOut)
{
    idOut=obstacleId;
    centerPointOut=centerPoint;
    sizeOut=size;
    yawAngleOut=yawAngle;
    return 1;
}


int RectangleObstacle2d::setParameters(unsigned int idIn, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn)
{
    return define(idIn,centerPointIn,sizeIn,yawAngleIn);
}


int RectangleObstacle2d::define(unsigned int id, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn)
{
    setObstacleType(2);

    obstacleId=id;
    centerPoint=centerPointIn;
    size=sizeIn;
    yawAngle=yawAngleIn;

    return 1;
}


double RectangleObstacle2d::implicitEquation(std::vector<double> pointIn, std::vector<double> robotDimensions)
{
    //http://math.stackexchange.com/questions/69099/equation-of-a-rectangle

    //cout<<"Rectangle implicit eq"<<endl;

    std::vector<double> pointRotated(2);

    pointRotated[0]=(pointIn[0]-centerPoint[0])*cos(yawAngle)        + (pointIn[1]-centerPoint[1])*sin(yawAngle);
    pointRotated[1]=(pointIn[0]-centerPoint[0])*(-1.0)*sin(yawAngle) + (pointIn[1]-centerPoint[1])*cos(yawAngle);


    double value=0.0;


    //value=pow((pointRotated[0]-centerPoint[0])/(radius[0]+robotDimensions[0]),2)+pow((pointRotated[1]-centerPoint[1])/(radius[1]+robotDimensions[1]),2)-1;

    double p=size[0]/2.0+robotDimensions[0]/2.0;
    double q=size[1]/2.0+robotDimensions[1]/2.0;

    value=abs(pointRotated[0]/p + pointRotated[1]/q ) + abs( pointRotated[0]/p - pointRotated[1]/q ) - 2.0;


    return value;
}





////////////////////// WorldMap /////////////////////
WorldMap::WorldMap()
{
    init();
    return;
}


WorldMap::~WorldMap()
{
    clear();
    return;
}

int WorldMap::init()
{
    return 1;
}

int WorldMap::clear()
{
    for(unsigned int i=0;i<obstaclesList.size();i++)
    {
        delete obstaclesList[i];
    }

    dimension.clear();
    initPoint.clear();

    return 1;
}



int WorldMap::setDimensions(std::vector<double> dimensionIn)
{
    //cout<<"MyMap dim="<<dimensionIn[0]<<";"<<dimensionIn[1]<<endl;
    if(dimensionIn.size()<dimension.size())
        return 0;

    for(unsigned int i=0;i<dimension.size();i++)
        dimension[i]=dimensionIn[i];

    return 1;
}


int WorldMap::getDimensions(vector<double> &dimensionOut)
{
    dimensionOut=dimension;
    return 1;
}


int WorldMap::getDimension(unsigned int &dimensionNum)
{
    dimensionNum=dimension.size();
    return 1;
}

int WorldMap::setInitPoint(std::vector<double> initPointIn)
{
    initPoint=initPointIn;
    return 1;
}

int WorldMap::getInitPoint(std::vector<double> &initPointOut)
{
    initPointOut=initPoint;
    return 1;
}



int WorldMap::getNumberObstacles()
{
    return obstaclesList.size();
}


int WorldMap::getHandleObstacle(unsigned int numObstacleI, ObstacleInMap** obstacleHandle)
{
    if(numObstacleI>=obstaclesList.size())
    {
        *obstacleHandle=NULL;
        return 0;
    }

    *obstacleHandle=obstaclesList[numObstacleI];

    return 1;
}


int WorldMap::findObstacle(unsigned int &numObstacleI, unsigned int idObstacle)
{
    for(unsigned int i=0;i<obstaclesList.size();i++)
    {
        //cout<<"id buscando="<<obstaclesList[i]->getObstacleId()<<endl;

        if(obstaclesList[i]->getObstacleId()==idObstacle)
        {
            numObstacleI=i;
            return 1;
        }
    }
    //cout<<"error finding obstacles inside WorldMap::findObstacle"<<endl;
    return 0;
}


bool WorldMap::existObstacle(unsigned int idObstacle)
{
    for(unsigned int i=0;i<obstaclesList.size();i++)
    {
        if(obstaclesList[i]->getObstacleId()==idObstacle)
        {
            return true;
        }
    }
    return false;
}



int WorldMap::implicitEquationObstacleI(double &implicitEqValOut, unsigned int numObstacleI, std::vector<double> pointIn, std::vector<double> robotDimensions)
{
    //cout<<"implicitEquationObstacleI"<<endl;
    if(numObstacleI>=obstaclesList.size())
        return 0;


    //cout<<obstaclesList.size()<<endl;
    //cout<<numObstacleI<<endl;

    //cout<<"aqui"<<endl;

    implicitEqValOut=obstaclesList[numObstacleI]->implicitEquation(pointIn,robotDimensions);

    //cout<<"yie"<<endl;

    return 1;
}

int WorldMap::deleteObstacle(unsigned int idObstacle)
{
    unsigned int obstacleI;

    //cout<<"here"<<endl;

    if(!findObstacle(obstacleI,idObstacle))
    {
        //cout<<"error finding obstacles inside WorldMap::deleteObstacle"<<endl;
        return 0;
    }

    //cout<<"aqui"<<endl;

    delete obstaclesList[obstacleI];

    obstaclesList.erase(obstaclesList.begin()+obstacleI);

    return 1;
}

int WorldMap::listIdObstacles()
{
#ifdef VERBOSE_SPACE_MAP
    cout<<"[WM] obstacles in map: ";
    for(unsigned int i=0;i<obstaclesList.size();i++)
    {
        cout<<obstaclesList[i]->getObstacleId()<<"; ";


    }
    cout<<endl;
#endif

    return 1;
}



////////////////////// WorldMap2d /////////////////////

WorldMap2d::WorldMap2d()
{
    init();

    return;
}


WorldMap2d::~WorldMap2d()
{
    clear();
    return;
}

int WorldMap2d::init()
{
    dimension.resize(2);
    return 1;
}

int WorldMap2d::clear()
{

    return 1;
}


int WorldMap2d::getHandleEllipseObstacle2d(unsigned int numObstacleI, EllipseObstacle2d **obstacleHandle)
{
    ObstacleInMap* ObstaclePtr;

    if(!getHandleObstacle(numObstacleI,&ObstaclePtr))
        return 0;

    *obstacleHandle=(EllipseObstacle2d*)ObstaclePtr;

    return 1;
}


int WorldMap2d::addEllipseObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn)
{
    if(existObstacle(idObstacle))
        return 0;


    EllipseObstacle2d* OneEllipsePtr=new EllipseObstacle2d;

    if(!OneEllipsePtr->define(idObstacle,centerPointIn,radiusIn,yawAngleIn))
        return 0;


    obstaclesList.push_back(OneEllipsePtr);

    //idObstacle=obstaclesList.size()-1;



/*
    vector<double> robotDimensions={0.5, 0.5};
    vector<double> pointIn={2.0, 2.0};

    obstaclesList[idObstacle]->implicitEquation(pointIn,robotDimensions);
*/


    return 1;
}


int WorldMap2d::updateEllipseObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn)
{

    if(!existObstacle(idObstacle))
        return 0;


    EllipseObstacle2d* OneEllipsePtr;

    //OneEllipsePtr=(EllipseObstacle2d*)obstaclesList[idObstacle];
    OneEllipsePtr=static_cast<EllipseObstacle2d*>(obstaclesList[idObstacle]);


    if(!OneEllipsePtr->define(idObstacle,centerPointIn,radiusIn,yawAngleIn))
        return 0;

    return 1;
}


int WorldMap2d::addRectangleObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn)
{
    if(existObstacle(idObstacle))
        return 0;

    RectangleObstacle2d* OneEllipsePtr=new RectangleObstacle2d;

    if(!OneEllipsePtr->define(idObstacle,centerPointIn,sizeIn,yawAngleIn))
    {
        return 0;
    }

    obstaclesList.push_back(OneEllipsePtr);

    idObstacle=obstaclesList.size()-1;

    return 1;
}


int WorldMap2d::updateRectangleObstacle2d(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn)
{
    if(!existObstacle(idObstacle))
        return 0;

    RectangleObstacle2d* OneEllipsePtr;

    OneEllipsePtr=static_cast<RectangleObstacle2d*>(obstaclesList[idObstacle]);

    if(!OneEllipsePtr->define(idObstacle,centerPointIn,sizeIn,yawAngleIn))
        return 0;

    return 1;
}



int WorldMap2d::listObstacles()
{
#ifdef VERBOSE_SPACE_MAP
    cout<<"[WM2d] obstacles in map: "<<endl;

    for(unsigned int i=0; i<obstaclesList.size();i++)
    {
        if(obstaclesList[i]->getObstacleType()==1)
        {
            //Elipse
            EllipseObstacle2d* ThisEllipse=static_cast<EllipseObstacle2d*>(obstaclesList[i]);

            //
            unsigned int idOut;
            std::vector<double> centerPointOut;
            std::vector<double> radiusOut;
            double yawAngleOut;
            ThisEllipse->getParameters(idOut,centerPointOut,radiusOut,yawAngleOut);

            cout<<"[WM2d] Ellipse id="<<idOut<<"; centerPoint="<<centerPointOut[0]<<";"<<centerPointOut[1]<<"; radius="<<radiusOut[0]<<";"<<radiusOut[1]<<"; yaw="<<yawAngleOut<<endl;

        }
        else if(obstaclesList[i]->getObstacleType()==2)
        {
            //Pole
        }


    }


#endif

    return 1;
}

