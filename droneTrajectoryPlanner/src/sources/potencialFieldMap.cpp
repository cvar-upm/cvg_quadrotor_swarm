//////////////////////////////////////////////////////
//  potencialFieldMap.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "potencialFieldMap.h"


using namespace std;


////////////////// ObstacleInPotencialMap ////////////////////////

ObstacleInPotencialMap::ObstacleInPotencialMap()
{
    init();

    return;
}


ObstacleInPotencialMap::~ObstacleInPotencialMap()
{
    clear();

    return;
}

int ObstacleInPotencialMap::init()
{
    obstacleAltitudeDef=OBSTACLE_ALTITUDE_DEF;
    obstaclePrecissionDef=OBSTACLE_PRECISSION_DEF;

    return 1;
}


int ObstacleInPotencialMap::clear()
{
    return 1;
}


int ObstacleInPotencialMap::define(double obstacleAltitudeDefIn, double obstaclePrecissionDefIn)
{
    obstacleAltitudeDef=obstacleAltitudeDefIn;
    obstaclePrecissionDef=obstaclePrecissionDefIn;

    return 1;
}



 int ObstacleInPotencialMap::calculateAltitudePotMap(double &valueAltitudeOut, double valueImplicitEquation)
{
    //double value=0.0;

    //value=implicitEquation(pointIn);

    if(valueImplicitEquation<0.0)
    {
         valueAltitudeOut=obstacleAltitudeDef/2.0;
         return 1;
    }

    valueAltitudeOut=obstacleAltitudeDef/(1+exp(obstaclePrecissionDef*valueImplicitEquation));

    return 1;
}





////////////////// EllipseObstacleInPotencialMap2d ////////////////////////
/*
double EllipseObstacleInPotencialMap2d::calculateAltitudePotMap(std::vector<double> pointIn)
{
    double valueImplicitEquation=TheEllipseObstacle2d->implicitEquation(pointIn);

    return ObstacleInPotencialMap::calculateAltitudePotMap(valueImplicitEquation);
}
*/





////////////////// PotencialFieldMap ////////////////////////

PotencialFieldMap::PotencialFieldMap()
{
    init();

    return;
}


PotencialFieldMap::~PotencialFieldMap()
{
    clear();
    return;
}

int PotencialFieldMap::init()
{
    pointInitAltitude=0.0;
    pointFinAltitude=0.0;


    return 1;
}

int PotencialFieldMap::clear()
{
    return 1;
}


int PotencialFieldMap::setDimension(unsigned int dimensionIn)
{
    dimension=dimensionIn;
    //cout<<"dim="<<dimension<<endl;

    pointInit.resize(dimension);
    pointFin.resize(dimension);


    return 1;
}


int PotencialFieldMap::setRobotDimensions(std::vector<double> robotDimensionsIn)
{
    robotDimensions=robotDimensionsIn;
    return 1;
}

int PotencialFieldMap::setRealMap(WorldMap* RealMapIn)
{
    RealMap=RealMapIn;
    //cout<<"Real map dime="<<RealMap->dimension.at(0)<<";"<<RealMap->dimension[1]<<endl;
    return 1;
}

int PotencialFieldMap::setPointsInitFin(std::vector<double> pointInitIn, std::vector<double> pointFinIn, double pointInitAltitudeIn, double pointFinAltitudeIn)
{
    pointInit=pointInitIn;
    pointFin=pointFinIn;

    if(pointInit.size()!=dimension && pointFin.size()!=dimension)
        return 0;


    if(pointFinAltitudeIn==0.0)
        pointFinAltitude=0.0;
    else
        pointFinAltitude=pointFinAltitudeIn;

    if(pointInitAltitudeIn==-1.0)
    {
        pointInitAltitude=0.0;
        for(unsigned int i=0;i<pointInit.size();i++)
        {
            pointInitAltitude+=pow(pointInit[i]-pointFin[i],2);
        }
        pointInitAltitude=sqrt(pointInitAltitude);
    }
    else
        pointInitAltitude=pointInitAltitudeIn;


    if(pointInitAltitude<pointFinAltitude)
        return 0;

    //cout<<"pointIn="<<pointInit[0]<<"; "<<pointInit[1]<<endl;
    //cout<<"pointFin="<<pointFin[0]<<"; "<<pointFin[1]<<endl;

    //cout<<"altitudes in and fin="<<pointInitAltitude<<"; "<<pointFinAltitude<<endl;


    if(!calculatePotMapCurve())
        return 0;



    return 1;
}


int PotencialFieldMap::setPointInit(std::vector<double> pointInitIn, double pointInitAltitudeIn)
{
//    cout<<"b0"<<endl;
    pointInit=pointInitIn;
//cout<<"b1"<<endl;
    if(pointInitAltitudeIn==-1.0)
    {
        //cout<<"b2"<<endl;
        pointInitAltitude=0.0;
        for(unsigned int i=0;i<pointInit.size();i++)
        {
            pointInitAltitude+=pow(pointInit[i]-pointFin[i],2);
        }
        //cout<<"b3"<<endl;
        pointInitAltitude=sqrt(pointInitAltitude);
    }
    else
        pointInitAltitude=pointInitAltitudeIn;
//cout<<"b"<<endl;
    if(!calculatePotMapCurve())
        return 0;
//cout<<"bb"<<endl;
    return 1;
}

int PotencialFieldMap::setPointFin(std::vector<double> pointFinIn, double pointFinAltitudeIn)
{
    //cout<<"b"<<endl;
    pointFin=pointFinIn;

    if(pointFinAltitudeIn==0.0)
        pointFinAltitude=0.0;
    else
        pointFinAltitude=pointFinAltitudeIn;
//cout<<"bn-1"<<endl;
    if(!calculatePotMapCurve())
        return 0;
    //cout<<"bn"<<endl;
    return 1;
}



int PotencialFieldMap::updateObstacleMap()
{
    ObstaclesList.resize(RealMap->getNumberObstacles());

#ifdef VERBOSE_POTENCIAL_FIELD_MAP
            cout<<"[PFM] (updateObstacleMap) number of obstacles in potencial field map="<<RealMap->getNumberObstacles()<<endl;
#endif

    return 1;
}


int PotencialFieldMap::calculateAltitudePotMapCurve(double &valueAltitudeOut, std::vector<double> pointIn)
{

    return 0;
}


int PotencialFieldMap::calculateAltitudePotMapObstacleI(double &valueAltitudeOut, std::vector<double> pointIn, unsigned int numObstacleI)
{
    double value=0.0;

    //cout<<"calculateAltitudePotMapObstacleI"<<endl;


    //getHandleObstacle(numObstacleI)

    //value=RealMap->obstaclesList[numObstacleI]->implicitEquation(pointIn,robotDimensions);

    if(!RealMap->implicitEquationObstacleI(value,numObstacleI,pointIn,robotDimensions))
        return 0;

    //cout<<"Inside pot field map:"<<value<<endl;



    //value=obstacleAltitudeDef/(1+exp(obstaclePrecissionDef*value));

    return ObstaclesList[numObstacleI].calculateAltitudePotMap(valueAltitudeOut,value);
}


int PotencialFieldMap::calculatePotMapCurve()
{
    return 0;
}



int PotencialFieldMap::calculateDistPotMap(double &distanceOut, std::vector<double> pointIn, std::vector<double> pointFin, bool useObstacles, bool useCurve)
{
    distanceOut=0.0;


    //end
    return 0;
}









////////////////// PotencialFieldMap2d ////////////////////////
PotencialFieldMap2d::PotencialFieldMap2d()
{
    init();
    return;
}

PotencialFieldMap2d::~PotencialFieldMap2d()
{
    clear();
    return;
}

int PotencialFieldMap2d::init()
{
    dimension=2;
    return 1;
}

int PotencialFieldMap2d::clear()
{
    return 1;
}

int PotencialFieldMap2d::calculatePotMapCurve()
{
    potFieldMapCurve.resize(dimension);

    potFieldMapCurve[0]= ( (pow(pointInit[0]-pointFin[0],2)+pow(pointInit[1]-pointFin[1],2)) / (pointInitAltitude-pointFinAltitude) );
    potFieldMapCurve[1]= ( (pow(pointInit[0]-pointFin[0],2)+pow(pointInit[1]-pointFin[1],2)) / (pointInitAltitude-pointFinAltitude) );

    return 1;
}


int PotencialFieldMap2d::setRealMap(WorldMap2d* RealMapIn)
{
    RealMap=RealMapIn;

    unsigned int dimensionIn;


    if(!RealMap->getDimension(dimensionIn))
        return 0;

    //cout<<"[inside potFieldMap2d] dimension real map="<<dimensionIn<<endl;

    if(!setDimension(dimensionIn))
        return 0;

    return 1;
}


int PotencialFieldMap2d::calculateAltitudePotMapCurve(double &valueAltitudeOut, std::vector<double> pointIn)
{
    //cout<<"PointFin="<<pointFin[0]<<"; "<<pointFin[1]<<endl;

    valueAltitudeOut=pointFinAltitude+pow((pointIn[0]-pointFin[0]),2)/potFieldMapCurve[0]+pow((pointIn[1]-pointFin[1]),2)/potFieldMapCurve[1];

    //cout<<"altitude curve="<<valueAltitudeOut<<endl;

    return 1;
}


int PotencialFieldMap2d::calculateDistPotMap(double &distanceOut, std::vector<double> pointInitIn, std::vector<double> pointFinIn, bool useObstacles, bool useCurve)
{
    //cout<<"aqui\n";

    //cout<<"calculateDistPotMap"<<endl;

    //Change of variable
    double angleAlpha=atan2(pointFinIn[1]-pointInitIn[1],pointFinIn[0]-pointInitIn[0]);
    double smin=0.0;
    double smax=sqrt( pow(pointFinIn[1]-pointInitIn[1],2)+pow(pointFinIn[0]-pointInitIn[0],2) );

    //Iteration vars
    double stepSize=STEP_SIZE_IN_INTEGRATION_S;
    int numSteps=floor((smax-smin)/stepSize);

    //cout<<"smax="<<smax<<endl;
    //cout<<"numSteps="<<numSteps<<endl;

    //Point to change the variables
    vector<double> pointSwap(dimension);

    //Altitudes for integration
    double altitudeInit;
    double altitudeFin;

    //Init algorithm
    distanceOut=0.0;
    double s=0.0;

    //Point of change the variables
    pointSwap[0]=pointInitIn[0]+s*cos(angleAlpha);
    pointSwap[1]=pointInitIn[1]+s*sin(angleAlpha);

    //Init loop
    altitudeInit=0.0;


    //cout<<"aqui"<<endl;

    //Potencial of curve
    if(useCurve)
    {
        double aux=0.0;
        if(!calculateAltitudePotMapCurve(aux,pointSwap))
            return 0;
        altitudeInit+=aux;
    }

    //cout<<"aqui"<<endl;
    //cout<<altitudeInit<<endl;

    //Potencial of obstacles
    if(useObstacles)
    {
        double aux=0.0;
        for(unsigned int numObstacleI=0;numObstacleI<ObstaclesList.size();numObstacleI++)
        {
            if(!calculateAltitudePotMapObstacleI(aux,pointSwap,numObstacleI))
                return 0;
            altitudeInit+=aux;
        }
    }


//cout<<"aqui"<<endl;
//cout<<altitudeInit<<endl;

    for(int numStepI=0;numStepI<numSteps;numStepI++)
    {
        //Take step
        s+=stepSize;

        //Old variables
        pointSwap[0]=pointInitIn[0]+s*cos(angleAlpha);
        pointSwap[1]=pointInitIn[1]+s*sin(angleAlpha);

        //ALtitude fin
        altitudeFin=0.0;

        //Potencial of curve
        if(useCurve)
        {
            double aux=0.0;
            if(!calculateAltitudePotMapCurve(aux,pointSwap))
                return 0;
            altitudeFin+=aux;
        }

        //Potencial of obstacles
        if(useObstacles)
        {
            double aux=0.0;
            for(unsigned int numObstacleI=0;numObstacleI<ObstaclesList.size();numObstacleI++)
            {
                if(!calculateAltitudePotMapObstacleI(aux,pointSwap,numObstacleI))
                    return 0;
                altitudeFin+=aux;
            }
        }

        //Longitud arc
        distanceOut+=sqrt( pow(altitudeFin-altitudeInit,2) + pow(stepSize,2) );

        //Update for next step
        altitudeInit=altitudeFin;
    }


    //Last step
    double lastDs=(smax-smin)-stepSize*numSteps;
    s+=lastDs;

    //Old variables
    pointSwap[0]=pointInitIn[0]+s*cos(angleAlpha);
    pointSwap[1]=pointInitIn[1]+s*sin(angleAlpha);

    //ALtitude fin
    altitudeFin=0.0;

    //Potencial of curve
    if(useCurve)
    {
        double aux=0.0;
        if(!calculateAltitudePotMapCurve(aux,pointSwap))
            return 0;
        altitudeFin+=aux;
    }

    //Potencial of obstacles
    if(useObstacles)
    {
        double aux=0.0;
        for(unsigned int numObstacleI=0;numObstacleI<ObstaclesList.size();numObstacleI++)
        {
            if(!calculateAltitudePotMapObstacleI(aux,pointSwap,numObstacleI))
                return 0;
            altitudeFin+=aux;
        }
    }

    //Longitud arc
    distanceOut+=sqrt( pow(altitudeFin-altitudeInit,2) + pow(lastDs,2) );


    //end
    return 1;
}
