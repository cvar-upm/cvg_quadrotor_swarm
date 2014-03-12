//////////////////////////////////////////////////////
//  trajectoryPlanner.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "trajectoryPlanner.h"

using namespace std;




Robot::Robot()
{
    id=0;
    centerPoint.clear();
    sizeDimensions.clear();
    return;
}

Robot::~Robot()
{
    centerPoint.clear();
    sizeDimensions.clear();
    return;
}

unsigned int Robot::getRobotId()
{
    return id;
}





Robot2d::Robot2d()
{
    yawAngle=0.0;
    return;
}

int Robot2d::getRobotParameters(unsigned int& idOut, std::vector<double>& centerPointOut, std::vector<double>& sizeDimensionsOut, double& yawAngleOut)
{
    idOut=id;
    centerPointOut=centerPoint;
    sizeDimensionsOut=sizeDimensions;
    yawAngleOut=yawAngle;
    return 1;
}


int Robot2d::setRobotParameters(unsigned int idIn, std::vector<double> centerPointIn, std::vector<double> sizeDimensionsIn, double yawAngleIn)
{
    id=idIn;
    centerPoint=centerPointIn;
    sizeDimensions=sizeDimensionsIn;
    yawAngle=yawAngleIn;
    return 1;
}






RobotTrajectoryPlanner::RobotTrajectoryPlanner()
{
    init();
    return;
}

RobotTrajectoryPlanner::~RobotTrajectoryPlanner()
{
    clear();
    return;
}


int RobotTrajectoryPlanner::init()
{
//    std::default_random_engine generator;
//    std::time_t result = std::time(NULL);
//    generator.seed(result);
//    std::uniform_real_distribution<double> distribution(50.0,500.0);

//    maxCostToReplan=MyAStar.maxCost/distribution(generator);


    maxCostToReplan=MyAStar.maxCost;

    return 1;
}

int RobotTrajectoryPlanner::clear()
{
    return 1;
}

int RobotTrajectoryPlanner::initMap(std::vector<double> dimensionMyMapIn, std::vector<double> initPointMyMapIn)
{
    if(!MyMap->setDimensions(dimensionMyMapIn))
        return 0;

    if(!MyMap->setInitPoint(initPointMyMapIn))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::initPRM(std::string prmConfigFile, std::string nodesListFile, std::string nodesRelationFile)
{
    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(prmConfigFile);
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
        return 0;

    std::string readingValue;
    pugi::xml_node prm = doc.child("prm");

    //Reading generation variable
    bool newPRM=false;
    readingValue=prm.child_value("generate");
    istringstream convertGenerateNewPRM(readingValue);
    convertGenerateNewPRM>>newPRM;

    if(newPRM)
    {
        pugi::xml_node config = prm.child("config");

        int numNodes, neigbourhood;

        //numNodes
        readingValue=config.child_value("numNodes");
        istringstream convertnumNodes(readingValue);
        convertnumNodes>>numNodes;

        //neigbourhood
        readingValue=config.child_value("neigbourhood");
        istringstream convertneigbourhood(readingValue);
        convertneigbourhood>>neigbourhood;

        //Generate PRM
        if(!MyPRM.configure(numNodes,neigbourhood))
            return 0;
        if(!MyPRM.generateFreePRM(*MyMap))
            return 0;

        //Subzones
        std::vector<double> initPointZone;
        std::vector<double> sizeZone;
        int numNodesSubzone;
        for(pugi::xml_node subzone = config.child("subzone"); subzone; subzone = subzone.next_sibling("subzone"))
        {
            //Num nodes subzone
            readingValue=subzone.child_value("numNodes");
            istringstream convertnumNodesSubzone(readingValue);
            convertnumNodesSubzone>>numNodesSubzone;

            //Center point
            readingValue=subzone.child_value("initPoint");
            istringstream convertinitPointZone(readingValue);
            while(1)
            {
                double initPointZoneAux;
                convertinitPointZone>>initPointZoneAux;
                if(convertinitPointZone==0)
                    break;
                initPointZone.push_back(initPointZoneAux);
            }

            //Size
            readingValue=subzone.child_value("size");
            istringstream convertSize(readingValue);
            while(1)
            {
                double sizeZoneAux;
                convertSize>>sizeZoneAux;
                if(convertSize==0)
                    break;
                sizeZone.push_back(sizeZoneAux);
            }

            //Generate subzone
            if(!MyPRM.addRandomNodes(numNodesSubzone,initPointZone,sizeZone))
                return 0;

        }
    }
    else
    {
        if(!MyPRM.load(nodesListFile,nodesRelationFile))
            return 0;
    }

    return 1;

}


int RobotTrajectoryPlanner::initPRM(std::string nodesListFile, std::string nodesRelationFile)
{

    if(!MyPRM.load(nodesListFile,nodesRelationFile))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::initPotFieldMap(vector<double> robotDimensionsIn)
{

    if(!MyPotencialFieldMap->setRobotDimensions(robotDimensionsIn))
        return 0;


    if(!MyPotencialFieldMap->setRealMap(MyMap)) //Aqui está el problema!!!
        return 0;

    //Añadimos obstaculos de MyMap a la lista de potencial
    if(!MyPotencialFieldMap->updateObstacleMap())
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::initAStar()
{
    //Set graph
    if(!MyAStar.setGraphToSearch(&MyPRM))
        return 0;


    //Set cost function
    if(!MyAStar.setPotencialFieldMap(MyPotencialFieldMap))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::initComponents(std::string configFile, std::string PRMNodesListFile, std::string PRMNodesRelationFile)
{
    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(configFile);
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        cout<<"I cannot open xml"<<endl;
        return 0;
    }

    std::string readingValue;

    ///Robot dimensions
    pugi::xml_node robot = doc.child("robot");

    vector<double> robotDimensionsIn;

    readingValue=robot.child_value("dimensions");
    istringstream convertrobotDimensions(readingValue);
    while(1)
    {
        double robotDimensionsAux;
        convertrobotDimensions>>robotDimensionsAux;
        if(convertrobotDimensions==0)
            break;
        robotDimensionsIn.push_back(robotDimensionsAux);
    }


    robotDimensions=robotDimensionsIn;

    //MAP
    pugi::xml_node config = doc.child("map").child("config");

    vector<double> dimensionMyMap;
    vector<double> initPointMyMap;

    //Reading generation variable
    readingValue=config.child_value("dimensions");
    istringstream convertdimensions(readingValue);
    while(1)
    {
        double dimensionMyMapAux;
        convertdimensions>>dimensionMyMapAux;
        if(convertdimensions==0)
            break;
        dimensionMyMap.push_back(dimensionMyMapAux);
    }

    //Reading generation variable
    readingValue=config.child_value("initPoint");
    istringstream convertinitPoint(readingValue);
    while(1)
    {
        double initPointMyMapAux;
        convertinitPoint>>initPointMyMapAux;
        if(convertinitPoint==0)
            break;
        initPointMyMap.push_back(initPointMyMapAux);
    }

    //MAP
    if(!initMap(dimensionMyMap,initPointMyMap))
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] unable to init map"<<endl;
#endif
        return 0;
    }

    //////PRM
    if(!initPRM(configFile,PRMNodesListFile,PRMNodesRelationFile))
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] unable to init prm"<<endl;
#endif
        return 0;
    }

    ///////Potencial field map
    if(!initPotFieldMap(robotDimensions))
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] unable to init pot field map"<<endl;
#endif
        return 0;
    }

    //////A-Star
    if(!initAStar())
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] unable to init A-Star"<<endl;
#endif
        return 0;
    }

    return 1;
}


int RobotTrajectoryPlanner::createMap(std::string mapConfigFile)
{
    return 0;
}


int RobotTrajectoryPlanner::findTrajectory(bool &isTrajectoryFound)
{
    return 0;
}


int RobotTrajectoryPlanner::setPoints(std::vector<double> pointInitIn, std::vector<double> pointFinIn)
{
    //Save points
    pointInit=pointInitIn;
    pointFin=pointFinIn;

    ///// Conectando al prm
    if(!MyPRM.addNode(pointInit))
        return 0;

    if(!MyPRM.addNode(pointFin))
        return 0;

    //Puntos de busqueda en el potencial field map
    if(!MyPotencialFieldMap->setPointsInitFin(pointInit,pointFin))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::setPointInit(std::vector<double> pointInitIn)
{
    //Save points//Añadimos obstaculos de MyMap a la lista de potencial
    if(!MyPotencialFieldMap->updateObstacleMap())
    {
        //cout<<"Error updating potential field map"<<endl;
        return 0;
    }
    pointInit=pointInitIn;

    ///// Conectando al prm
    if(!MyPRM.addNode(pointInit))
        return 0;

    //Puntos de busqueda en el potencial field map
    //if(!MyPotencialFieldMap.setPointsInitFin(pointInit,pointFin))
    //    return 0;

    if(!MyPotencialFieldMap->setPointInit(pointInit))
        return 0;

    return 1;
}

int RobotTrajectoryPlanner::setPointFin(std::vector<double> pointFinIn)
{
    //Save points
    pointFin=pointFinIn;

    ///// Conectando al prm
    if(!MyPRM.addNode(pointFin))
        return 0;

    //Puntos de busqueda en el potencial field map
    //if(!MyPotencialFieldMap.setPointsInitFin(pointInit,pointFin))
    //    return 0;

    if(!MyPotencialFieldMap->setPointFin(pointFin))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::deleteObstacle(unsigned int idObstacle)
{
    if(!MyMap->deleteObstacle(idObstacle))
    {
        //cout<<" -error deleting obstacle "<<idObstacle<<" inside RobotTrajectoryPlanner::deleteObstacle"<<endl;
        return 0;
    }
    //Borramos obstaculos de MyMap a la lista de potencial
    if(!MyPotencialFieldMap->updateObstacleMap())
        return 0;

    return 1;
}



int RobotTrajectoryPlanner::clearQuadRotors()
{
    for(unsigned int i=0;i<othersRobotPose.size();i++)
    {
        if(!deleteObstacle(othersRobotPose[i]->getRobotId()))
        {
            //cout<<"error deleting obstacle inside RobotTrajectoryPlanner::clearQuadRotors"<<endl;
            //return 0;
        }
        delete othersRobotPose[i];
    }

    othersRobotPose.clear();


    return 1;
}


////UNUSED!!!
int RobotTrajectoryPlanner::checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn)
{
    return 0;
}


int RobotTrajectoryPlanner::checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn, std::vector<double> robotPointIn)
{
    return 0;
}


int RobotTrajectoryPlanner::getTrajectory(std::vector< std::vector<double> > &trajectoryFoundOut)
{
    if(!MyAStar.getSolution(trajectoryFound))
        return 0;

    trajectoryFoundOut=trajectoryFound;

    return 1;
}


int RobotTrajectoryPlanner::savePRM(std::string nodesList, std::string nodesRelation)
{
    //PRM
    if(!MyPRM.save(nodesList,nodesRelation))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::savePlannerResult(std::string solutionNodes, std::string simplifSolNodes)
{
    //A-Star
    if(!MyAStar.saveSolutionNodes(solutionNodes,simplifSolNodes))
        return 0;
    return 1;
}

int RobotTrajectoryPlanner::saveResult(std::string nodesList, std::string nodesRelation, std::string solutionNodes, std::string simplifSolNodes)
{
    //PRM
    if(!MyPRM.save(nodesList,nodesRelation))
        return 0;

    //A-Star
    if(!MyAStar.saveSolutionNodes(solutionNodes,simplifSolNodes))
        return 0;

    return 1;
}


int RobotTrajectoryPlanner::distancePointToRect(double &distance, std::vector<double> &pointProjected, std::vector<double> pointSearch, std::vector<double> point1, std::vector<double> point2)
{
    return 0;
}








RobotTrajectoryPlanner2d::RobotTrajectoryPlanner2d()
{
    //Polymorphism
    //WorldMap
    WorldMap2d* MyMap2d=new WorldMap2d;
    MyMap=static_cast<WorldMap2d*>(MyMap2d);
    //MyPotencialFieldMap
    PotencialFieldMap2d* MyPotencialFieldMap2d=new PotencialFieldMap2d;
    MyPotencialFieldMap=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap2d);


    return;
}


RobotTrajectoryPlanner2d::~RobotTrajectoryPlanner2d()
{
    //Be tidy
    //WorldMap
    WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
    delete MyMap2d;

    //PotentialFieldMap
    PotencialFieldMap2d* MyPotencialFieldMap2d=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap);
    delete MyPotencialFieldMap2d;

    return;
}



int RobotTrajectoryPlanner2d::init()
{
    if(!RobotTrajectoryPlanner::init())
        return 0;

    othersRobotPose.clear();
    return 1;
}

int RobotTrajectoryPlanner2d::clear()
{
    if(!RobotTrajectoryPlanner::clear())
        return 0;

    return 1;
}


int RobotTrajectoryPlanner2d::initPotFieldMap(vector<double> robotDimensionsIn)
{
    PotencialFieldMap2d* MyPotencialFieldMap2d=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap);
    if(!MyPotencialFieldMap2d->setRobotDimensions(robotDimensionsIn))
        return 0;

    WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
    if(!MyPotencialFieldMap2d->setRealMap(MyMap2d)) //Aqui estába el problema!!!
        return 0;

    //Añadimos obstaculos de MyMap a la lista de potencial
    if(!MyPotencialFieldMap2d->updateObstacleMap())
        return 0;

    return 1;
}


int RobotTrajectoryPlanner2d::createMap(std::string mapConfigFile)
{
    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(mapConfigFile);
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
        return 0;

    //Obstacles!
    int idObstacle;
    double obstYawAngle;
    vector<double> obstCenterPoint(2);
    //Polos
    vector<double> obstRadius(2);
    //Rectangle
    vector<double> obstSize(2);

    pugi::xml_node obstacles = doc.child("map").child("obstaclesInMap");

    //Reading Rectangles!!
    for(pugi::xml_node rectangle = obstacles.child("rectangle"); rectangle; rectangle = rectangle.next_sibling("rectangle"))
    {
        std::string readingValue;

        //Id obstacle
        readingValue=rectangle.child_value("id");
        istringstream convertId(readingValue);
        convertId>>idObstacle;

        //Center point
        readingValue=rectangle.child_value("centerPoint");
        istringstream convertCenterPoint(readingValue);
        for(unsigned int i=0;i<2;i++)
        {
            convertCenterPoint>>obstCenterPoint[i];
        }

        //Size
        readingValue=rectangle.child_value("size");
        istringstream convertSize(readingValue);
        for(unsigned int i=0;i<2;i++)
        {
            convertSize>>obstSize[i];
        }

        //Yaw Angle
        readingValue=rectangle.child_value("yawAngle");
        istringstream convertYawAngle(readingValue);
        convertYawAngle>>obstYawAngle;

        //Add obstacle
        WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
        if(!MyMap2d->addRectangleObstacle2d(idObstacle,obstCenterPoint,obstSize,obstYawAngle))
            return 0;
    }

    //Reading Ellipses!!
    for(pugi::xml_node ellipse = obstacles.child("ellipse"); ellipse; ellipse = ellipse.next_sibling("ellipse"))
    {
        std::string readingValue;

        //Id obstacle
        readingValue=ellipse.child_value("id");
        istringstream convertId(readingValue);
        convertId>>idObstacle;

        //Center point
        readingValue=ellipse.child_value("centerPoint");
        istringstream convertCenterPoint(readingValue);
        for(unsigned int i=0;i<2;i++)
        {
            convertCenterPoint>>obstCenterPoint[i];
        }

        //Radius
        readingValue=ellipse.child_value("radius");
        istringstream convertSize(readingValue);
        for(unsigned int i=0;i<2;i++)
        {
            convertSize>>obstRadius[i];
        }

        //Yaw Angle
        readingValue=ellipse.child_value("yawAngle");
        istringstream convertYawAngle(readingValue);
        convertYawAngle>>obstYawAngle;

        //Add obstacle
        WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
        if(!MyMap2d->addEllipseObstacle2d(idObstacle,obstCenterPoint,obstRadius,obstYawAngle))
            return 0;
    }
    return 1;
}


//Ellipse
int RobotTrajectoryPlanner2d::setEllipse(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> radiusIn, double yawAngleIn)
{
    WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
    if(!MyMap2d->existObstacle(idObstacle))
    {
        if(!MyMap2d->addEllipseObstacle2d(idObstacle,centerPointIn,radiusIn,yawAngleIn))
            return 0;
    }
    else
    {
        if(!MyMap2d->updateEllipseObstacle2d(idObstacle,centerPointIn,radiusIn,yawAngleIn))
            return 0;
    }

    //Añadimos obstaculos de MyMap a la lista de potencial
    PotencialFieldMap2d* MyPotencialFieldMap2d=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap);
    if(!MyPotencialFieldMap2d->updateObstacleMap())
        return 0;

    return 1;
}


//Rectangle
int RobotTrajectoryPlanner2d::setRectangle(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> sizeIn, double yawAngleIn)
{
    WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
    if(!MyMap2d->existObstacle(idObstacle))
    {
        if(!MyMap2d->addRectangleObstacle2d(idObstacle,centerPointIn,sizeIn,yawAngleIn))
            return 0;
    }
    else
    {
        if(!MyMap2d->updateRectangleObstacle2d(idObstacle,centerPointIn,sizeIn,yawAngleIn))
            return 0;
    }

    //Añadimos obstaculos de MyMap a la lista de potencial
    PotencialFieldMap2d* MyPotencialFieldMap2d=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap);
    if(!MyPotencialFieldMap2d->updateObstacleMap())
        return 0;

    return 1;
}



//Other QR
int RobotTrajectoryPlanner2d::setQuadRotor(unsigned int idObstacle, std::vector<double> centerPointIn, std::vector<double> qrSizeIn, double yawAngleIn)
{
    //Store in the robots list!!

    //Find if the robot already exists on the list
    int robotI=-1;
    for(unsigned int i=0;i<othersRobotPose.size();i++)
    {
        if(othersRobotPose[i]->getRobotId()==idObstacle)
        {
            robotI=i;
            break;
        }
    }
    if(robotI==-1)
    {
        //El robot todavia no está en la lista
        Robot2d* RobotAux=new Robot2d;
        RobotAux->setRobotParameters(idObstacle,centerPointIn,qrSizeIn,yawAngleIn);
        //
        othersRobotPose.push_back(RobotAux);

    }
    else
    {
        //update
        Robot2d* RobotAux=static_cast<Robot2d*>(othersRobotPose[robotI]);
        RobotAux->setRobotParameters(idObstacle,centerPointIn,qrSizeIn,yawAngleIn);
    }


    /*
    if(!MyMap.existObstacle(idObstacle))
    {
        if(!MyMap.addEllipseObstacle2d(idObstacle,centerPointIn,qrSizeIn,yawAngleIn))
            return 0;
    }
    else
    {
        if(!MyMap.updateEllipseObstacle2d(idObstacle,centerPointIn,qrSizeIn,yawAngleIn))
            return 0;
    }

    //Añadimos obstaculos de MyMap a la lista de potencial
    if(!MyPotencialFieldMap.updateObstacleMap())
        return 0;

    */

    return 1;
}



int RobotTrajectoryPlanner2d::findTrajectory(bool& isTrajectoryFound)
{
    //Return flag
    isTrajectoryFound=false;

    //Check points
    if(pointInit.size()>0 && pointFin.size()>0 && pointInit.size()==pointFin.size())
    {
        //Continue
    }
    else
    {
        //cout<<"error adding points"<<endl;
        return 0;
    }

    //Set in the map the robots that are near
    std::vector<unsigned int> robotObstacleIdAdded;
    unsigned int idDroneSoc;
    vector<double> centerPointDroneSoc(2);
    vector<double> sizeDimensionsDroneSoc(2);
    double yawAngleDroneSoc;
    for(unsigned int i=0;i<othersRobotPose.size();i++)
    {
        Robot2d* Robot2dAux=static_cast<Robot2d*>(othersRobotPose[i]);
        Robot2dAux->getRobotParameters(idDroneSoc,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc);
        double distance=0.0;
        for(unsigned int j=0;j<centerPointDroneSoc.size();j++)
        {
            distance+=pow(centerPointDroneSoc[j]-pointInit[j],2);
        }
        distance=sqrt(distance);

        if(distance<=DISTANCE_NEAR_ROBOT)
        {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
            cout<<"[RTP] (findTrajectory) robot to be considered! id="<<idDroneSoc<<". pose="<<centerPointDroneSoc[0]<<";"<<centerPointDroneSoc[1]<<". Size="<<sizeDimensionsDroneSoc[0]<<";"<<sizeDimensionsDroneSoc[1]<<"; Yaw="<<yawAngleDroneSoc<<endl;
#endif

            robotObstacleIdAdded.push_back(idDroneSoc+NUMBER_INIT_ROBOT_ID);
            WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
            if(!MyMap2d->existObstacle(idDroneSoc+NUMBER_INIT_ROBOT_ID))
            {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
                cout<<"[RTP] (findTrajectory) adding obstacle with ID="<<idDroneSoc+NUMBER_INIT_ROBOT_ID<<endl;
#endif
                if(!MyMap2d->addEllipseObstacle2d(idDroneSoc+NUMBER_INIT_ROBOT_ID,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
                    return 0;
            }
            else
            {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
                cout<<"[RTP] (findTrajectory) updating obstacle with ID="<<idDroneSoc+NUMBER_INIT_ROBOT_ID<<endl;
#endif
                if(!MyMap2d->updateEllipseObstacle2d(idDroneSoc+NUMBER_INIT_ROBOT_ID,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
                    return 0;
            }
        }

    }

#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    {
    WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
    //MyMap2d->listIdObstacles();
    MyMap2d->listObstacles();
    }
#endif



    //Añadimos obstaculos de MyMap a la lista de potencial
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (findTrajectory) we are going to update pot field map"<<endl;
#endif
    PotencialFieldMap2d* MyPotencialFieldMap2d=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap);
    if(!MyPotencialFieldMap2d->updateObstacleMap())
    {
        //cout<<"Error updating potential field map inside RobotTrajectoryPlanner::findTrajectory"<<endl;
        return 0;
    }


    //Display obstacles
/*
    for(unsigned int i=0;i<MyPotencialFieldMap.RealMap->obstaclesList.size();i++)
    {
        cout<<"obstacle i="<<MyPotencialFieldMap.RealMap->obstaclesList.at(i)->obstacleId<<" in MyPotencialFieldMap.RealMap->obstaclesList"<<endl;
    }
*/


    //Search
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (findTrajectory) we are going to search in the A-Star"<<endl;
#endif
    if(!MyAStar.search(pointInit,pointFin,isTrajectoryFound))
    {
        //cout<<"Error searching inside RobotTrajectoryPlanner::findTrajectory"<<endl;
        return 0;
    }



    //We delete the robots added!
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (findTrajectory) we are going to delete robots added"<<endl;
#endif
    for(unsigned int i=0;i<robotObstacleIdAdded.size();i++)
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] (findTrajectory) deleting robot obstacle "<<robotObstacleIdAdded[i]<<endl;
#endif
        if(!deleteObstacle(robotObstacleIdAdded[i]))
            return 0;
    }


    //Borramos obstaculos de MyMap a la lista de potencial
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (findTrajectory) we are going to update pot field map"<<endl;
#endif
    if(!MyPotencialFieldMap2d->updateObstacleMap())
    {
        //cout<<"Error updating potential field map"<<endl;
        return 0;
    }
    return 1;
}



////UNUSED!!!
int RobotTrajectoryPlanner2d::checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn)
{
    //cout<<"in"<<endl;

    freeColission=false;


    if(TrajectoryIn.size()<1)
        return 0;



    //JL to do!!!
    //Añadir robots que estan cerca!!
    //Set in the map the robots that are near
    std::vector<unsigned int> robotObstacleIdAdded;
    unsigned int idDroneSoc;
    vector<double> centerPointDroneSoc(2);
    vector<double> sizeDimensionsDroneSoc(2);
    double yawAngleDroneSoc;
    for(unsigned int i=0;i<othersRobotPose.size();i++)
    {
        Robot2d* Robot2dAux=static_cast<Robot2d*>(othersRobotPose[i]);
        Robot2dAux->getRobotParameters(idDroneSoc,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc);
        double distance=0.0;
        for(unsigned int j=0;j<centerPointDroneSoc.size();j++)
        {
            distance+=pow(centerPointDroneSoc[j]-pointInit[j],2);
        }
        distance=sqrt(distance);

        if(distance<=DISTANCE_NEAR_ROBOT)
        {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
            cout<<"[RTP] (checkTrajectory) robot to be considered! id="<<idDroneSoc<<endl;
#endif
            robotObstacleIdAdded.push_back(idDroneSoc+NUMBER_INIT_ROBOT_ID);

            WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
            if(!MyMap2d->existObstacle(idDroneSoc+NUMBER_INIT_ROBOT_ID))
            {
                if(!MyMap2d->addEllipseObstacle2d(idDroneSoc+NUMBER_INIT_ROBOT_ID,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
                    return 0;
            }
            else
            {
                if(!MyMap2d->updateEllipseObstacle2d(idDroneSoc+NUMBER_INIT_ROBOT_ID,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
                    return 0;
            }
        }

    }




    double costTrajectory=0.0;
    double costI=0.0;

    vector<double> node1;
    //unsigned int node1Id;
    vector<double> node2;
    //unsigned int node2Id;




    node1=TrajectoryIn[0];


    //cout<<"aqui"<<endl;


    for(unsigned int i=1;i<TrajectoryIn.size();i++)
    {
        node2=TrajectoryIn[i];

        if(!MyAStar.postCost(costI,node1,node2))
            return 0;

        //cout<<"aqui"<<i<<endl;


        costTrajectory+=costI;


        node1=node2;


    }

    if(costTrajectory<MyAStar.maxCost)
    {
        freeColission=true;
    }




    //JL to do!!!
    //Borrar robots añadidos!!
    for(unsigned int i=0;i<robotObstacleIdAdded.size();i++)
    {
        if(!deleteObstacle(robotObstacleIdAdded[i]))
            return 0;
    }



    return 1;
}



int RobotTrajectoryPlanner2d::checkTrajectory(bool &freeColission, std::vector< std::vector<double> > TrajectoryIn, std::vector<double> robotPointIn)
{

    freeColission=false;


    //cout<<"CheckTrajectory function"<<endl;


    if(TrajectoryIn.size()<2) //At least point init and fin
        return 0;



    //Añadir robots que estan cerca!!
    //Set in the map the robots that are near
    std::vector<unsigned int> robotObstacleIdAdded;
    unsigned int idDroneSoc;
    vector<double> centerPointDroneSoc(2);
    vector<double> sizeDimensionsDroneSoc(2);
    double yawAngleDroneSoc;
    for(unsigned int i=0;i<othersRobotPose.size();i++)
    {
        Robot2d* Robot2dAux=static_cast<Robot2d*>(othersRobotPose[i]);
        Robot2dAux->getRobotParameters(idDroneSoc,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc);
        double distance=0.0;
        for(unsigned int j=0;j<centerPointDroneSoc.size();j++)
        {
            distance+=pow(centerPointDroneSoc[j]-pointInit[j],2);
        }
        distance=sqrt(distance);

        if(distance<=DISTANCE_NEAR_ROBOT)
        {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
            cout<<"[RTP] (checkTrajectory) robot to be considered! id="<<idDroneSoc<<". Pose="<<centerPointDroneSoc[0]<<";"<<centerPointDroneSoc[1]<<". Size="<<sizeDimensionsDroneSoc[0]<<";"<<sizeDimensionsDroneSoc[1]<<endl;
#endif

            robotObstacleIdAdded.push_back(idDroneSoc+NUMBER_INIT_ROBOT_ID);

            WorldMap2d* MyMap2d=static_cast<WorldMap2d*>(MyMap);
            if(!MyMap2d->existObstacle(idDroneSoc+NUMBER_INIT_ROBOT_ID))
            {
                if(!MyMap2d->addEllipseObstacle2d(idDroneSoc+NUMBER_INIT_ROBOT_ID,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
                    return 0;
            }
            else
            {
                //cout<<"NO ENTRO NUNCA!!!"<<endl;
                if(!MyMap2d->updateEllipseObstacle2d(idDroneSoc+NUMBER_INIT_ROBOT_ID,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
                    return 0;
            }
        }

    }


    //Añadimos obstaculos de MyMap a la lista de potencial
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (checkTrajectory) we are going to update pot field map"<<endl;
#endif
    PotencialFieldMap2d* MyPotencialFieldMap2d=static_cast<PotencialFieldMap2d*>(MyPotencialFieldMap);
    if(!MyPotencialFieldMap2d->updateObstacleMap())
    {
        //cout<<"Error updating potential field map inside RobotTrajectoryPlanner::findTrajectory"<<endl;
        return 0;
    }



    //Display obstacles
/*
    for(unsigned int i=0;i<MyPotencialFieldMap.RealMap->obstaclesList.size();i++)
    {
        cout<<"(checkTrajectory) obstacle i="<<MyPotencialFieldMap.RealMap->obstaclesList.at(i)->obstacleId<<" in MyPotencialFieldMap.RealMap->obstaclesList"<<endl;
    }
*/


    vector<double> node1;
    //unsigned int node1Id;
    vector<double> node2;
    //unsigned int node2Id;



    //Find part of the trajectory where is working
    unsigned int partTrajectory=0;
    std::vector<double> pointProjected;


    if(TrajectoryIn.size()>1)
    {
        //cout<<" point init (pose) "<<": "<<pointInit[0]<<"; "<<pointInit[1]<<endl;

        //cout<<"here"<<endl;
        double minDistance=99999.9;
        bool ok=false;
        for(unsigned int i=0;i<TrajectoryIn.size()-1;i++)
        {

            //cout<<" Trajectory point "<<i<<": "<<TrajectoryIn[i][0]<<"; "<<TrajectoryIn[i][1]<<endl;

            double distance=0.0;

            //cout<<"here"<<endl;
            if(distancePointToRect(distance,pointProjected,robotPointIn,TrajectoryIn[i],TrajectoryIn[i+1]))
            {
                //cout<<" distance="<<distance<<endl;

                if(distance<minDistance)
                {
                    minDistance=distance;
                    node1=pointProjected;
                    partTrajectory=i+1;
                    ok=true;
                }

            }
            else
            {
                //cout<<" error calculando distancePointToRect en checkTrajectory"<<endl;
            }

        }

        //cout<<" (inside RobotTrajectoryPlanner::checkTrajectory) segmento de la trajectoria="<<partTrajectory<<endl;

        if(!ok)
        {
            //cout<<" La hemos liado!!!!!"<<endl;
            return 0;
        }

    }

    //cout<<" point projected="<<node1[0]<<"; "<<node1[1]<<endl;


    //cout<<"aqui"<<endl;

    double costTrajectory=0.0;
    double costI=0.0;

    for(unsigned int i=partTrajectory;i<TrajectoryIn.size();i++)
    {
        node2=TrajectoryIn[i];

        if(!MyAStar.postCost(costI,node1,node2))
        {
            //cout<<"ha fallado MyAStar.postCost"<<endl;
            return 0;
        }

        //cout<<"aqui"<<i<<endl;


        costTrajectory+=costI;

        //cout<<"cost="<<costI<<"; i="<<i<<endl;


        node1=node2;


    }

    //cout<<"costTrajectory="<<costTrajectory<<endl;

    if(costTrajectory<maxCostToReplan)
    {
        freeColission=true;
    }
    else
    {
        //cout<<"Part trajectory="<<partTrajectory<<endl;
    }


    //JL to do!!!
    //Borrar robots añadidos!!
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (checkTrajectory) we are going to delete robots added"<<endl;
#endif
    for(unsigned int i=0;i<robotObstacleIdAdded.size();i++)
    {
        if(!deleteObstacle(robotObstacleIdAdded[i]))
            return 0;
    }

    //Borramos obstaculos de MyMap a la lista de potencial
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (checkTrajectory) we are going to update pot field map"<<endl;
#endif
    if(!MyPotencialFieldMap2d->updateObstacleMap())
    {
        //cout<<"Error updating potential field map"<<endl;
        return 0;
    }
    return 1;
}


int RobotTrajectoryPlanner2d::distancePointToRect(double &distance, std::vector<double> &pointProjected, std::vector<double> pointSearch, std::vector<double> point1, std::vector<double> point2)
{
    //cout<<"In function distancePointToRect"<<endl;

    //cout<<"aqui"<<endl;
    distance=0.0;
    pointProjected.clear();

    //cout<<"aqui"<<endl;

    //Previous checkings
    //cout<<" previous checkings"<<endl;
    if(point1.size()!=point2.size() || pointSearch.size()!=point1.size() || pointSearch.size()<2)
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect) previous checkings"<<endl;
#endif
        return 0;
    }

//cout<<"aqui"<<endl;

    if(point1==point2)
    {
        pointProjected=point1;

        for(unsigned int i=0;i<point1.size();i++)
            distance+=pow(pointProjected[i]-pointSearch[i],2);
        distance=sqrt(distance);

#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect) Points are similar!!"<<endl;
#endif

        return 1;

    }

//cout<<"aqui"<<endl;

    if(point1[0]!=point2[0])
    {

        //Calculate distance
        double a=(point1[1]-point2[1])/(point1[0]-point2[0]);
        double b=-1;
        double c=point1[1]-a*point1[0];


        distance=abs( a*pointSearch[0]+b*pointSearch[1]+c ) / sqrt(a*a+b*b);
        //cout<<" distance="<<distance<<endl;

//cout<<"aqui"<<endl;

        //Point projected
        std::vector<double> u(2);
        u[0]=1.0/sqrt(a*a+b*b)*(-b);
        u[1]=1.0/sqrt(a*a+b*b)*(a);
//cout<<"aqui"<<endl;
        double d1=0.0;
        for(unsigned int i=0;i<u.size();i++)
        {
            d1+=(pointSearch[i]-point1[i])*u[i];
        }

        for(unsigned int i=0;i<point1.size();i++)
        {
            pointProjected.push_back(point1[i]+d1*u[i]);

            //cout<<"  PointProjected "<<i<<"="<<pointProjected[i]<<endl;
        }
//cout<<"aqui"<<endl;

        //Point projected between two points
        double distApP0=pow(point1[0]-pointProjected[0],2)+pow(point1[1]-pointProjected[1],2);
        double distApP1=pow(point2[0]-pointProjected[0],2)+pow(point2[1]-pointProjected[1],2);
        double distP0P1=pow(point2[0]-point1[0],2)+pow(point2[1]-point1[1],2);

#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect)  distApP0="<<distApP0<<endl;
        cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect)  distApP1="<<distApP1<<endl;
        cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect)  distP0P1="<<distP0P1<<endl;
#endif

//cout<<"aqui"<<endl;
        double tol=TOL_FINDING_DISTANCE_POINT_TO_RECT; //error
        if(distApP0<=distP0P1+tol && distApP1<=distP0P1+tol)
        //if(abs(distApP0-distP0P1)<=+tol && abs(distApP1-distP0P1)<=+tol)
        {
            return 1;
        }
        else
        {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
            cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect) dif 1="<<distApP0-distP0P1<<endl;
            cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect) dif 2="<<distApP1-distP0P1<<endl;
#endif
            return 0;
        }


    }


    /////// JL TODO!!!!!!
    if(point1[0]==point2[0])
    {
#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
        cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect) Special condition!!!"<<endl;
#endif
        return 0;


    }

#ifdef VERBOSE_ROBOT_TRAJECTORY_PLANNER
    cout<<"[RTP] (RobotTrajectoryPlanner2d::distancePointToRect) No conditions!!"<<endl;
#endif
    return 0;
}
