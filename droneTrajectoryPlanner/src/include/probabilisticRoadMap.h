//////////////////////////////////////////////////////
//  probabilisticRoadMap.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 23, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef PROBABILISTIC_ROAD_MAP_H
#define PROBABILISTIC_ROAD_MAP_H





//C Math
//pow(), sqrt(), floor()
#include <cmath>

//C Standart Lib
//std::srand, std::rand
#include <cstdlib>

//C Time
//std::time
#include <ctime>


//I/O Stream
//std::cout
#include <iostream>

//Vector
//std::vector
#include <vector>

//Algorithm functions
//std::sort, std::find
#include <algorithm>

//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>




//Space map
#include "spaceMap.h"



/////////////////////////////////////////
// Class DiscreteGraph
//
//   Description
//
/////////////////////////////////////////
class DiscreteGraph
{
protected:
    //Configuration
    unsigned int numNodes;
    unsigned int nodesNeighborhood;
    unsigned int dimension;


protected:
    bool isGeneratedBool;


protected:
    //Nodos
    std::vector< std::vector<double> > nodesList;

    //Relacion entre nodos
    std::vector< std::vector<int> > nodesRelation;


public:
    DiscreteGraph();
    ~DiscreteGraph();

    int init();
    int clear();


public:
    //int configure(int numNodesIn, int nodesNeighborhoodIn, int dimensionIn);
    int configure(int numNodesIn, int nodesNeighborhoodIn);


public:

    bool isNodeInTheGraph(unsigned int nodeId);
    bool isNodeInTheGraph(std::vector<double> nodeIn);

    int findNode(unsigned int &nodeIdOut, std::vector<double> nodeIn); //Find a node id in the graph using the value of the node
    int findNode(std::vector<double> &nodeOut, unsigned int nodeId); //Find a node in the graph using its id

    int findNeighbordsNodes(int nodeI, std::vector<unsigned int> &neighborsNodeI);

    bool areNeighbordsNodes(int nodeI, int nodeJ);


public:

    int display();
    int save(std::string nameNodesList, std::string nameNodesRelation);
    //TODO JL
    int load(std::string nameNodesList, std::string nameNodesRelation);


};


/////////////////////////////////////////
// Class ProbabilisticRoadMap
//
//   Description
//
/////////////////////////////////////////
class ProbabilisticRoadMap : public DiscreteGraph
{

public:
    ProbabilisticRoadMap();
    ~ProbabilisticRoadMap();


public:
    int init();
    int clear();


public:
    int generateFreePRM(WorldMap &mapIn);


protected:
    double distanceBetweenNodes(std::vector<double> node1, std::vector<double> node2, int typeDist);


public:
    int addNode(std::vector<double> nodeIn);


public:
    int addRandomNode(std::vector<double> initPointZone, std::vector<double> sizeZone);

    int addRandomNodes(unsigned int numNodesAdd, std::vector<double> initPoint, std::vector<double> size);

};











#endif
