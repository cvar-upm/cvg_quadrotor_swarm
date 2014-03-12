//////////////////////////////////////////////////////
//  discreteSearch.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef DISCRETE_SEARCH_H
#define DISCRETE_SEARCH_H



//I/O stream
//std::cout
#include <iostream>

//Vector
//std::vector
#include <vector>

//Algorithm
//std::reverse
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


//Time
#include <ctime>

//Uniform random generation
#include <random>

//PRM
#include "probabilisticRoadMap.h"

//Potencial field map
#include "potencialFieldMap.h"



//Const
const double MAX_COST_A_STAR=1.0e09;
const double H_OVERCOST=10.0;




/////////////////////////////////////////
// Class AStar
//
//   Description
//
/////////////////////////////////////////
class AStar
{

protected:
    DiscreteGraph *GraphToSearch;


protected:
    std::vector< std::vector<double> > solutionNodes;
public:
    int getSolution(std::vector< std::vector<double> > &solutionNodesOut);


protected:
    //Nodos
    std::vector<double> nodeInit;
    std::vector<double> nodeFin;

    unsigned int nodeInitId;
    unsigned int nodeFinId;



protected:
    //Matriz de secuenciamiento de acciones
    std::vector< std::vector<unsigned int> > secuenciamientoAcciones; //[identif nodo 1; identif nodo 2]

    //Matriz de almacenamiento de estados visitados
    std::vector<unsigned int> statesVisitedStates;
    std::vector< std::vector<double> > statesVisitedPotencial; //[f=g+h, g, h]

    //matriz de estados ya evaluados y ya expandidos
    std::vector<unsigned int> statesClosedStates;
    std::vector< std::vector<double> > statesClosedPotencial; //[f=g+h, g, h]

    //Matriz de estados abiertos. Todavia no evaluados
    std::vector<unsigned int> statesOpenStates;
    std::vector< std::vector<double> > statesOpenPotencial; //[f=g+h, g, h]


    //Matriz de almacenamiento de allStates
    std::vector<unsigned int> statesAllStates;
    std::vector< std::vector<double> > statesAllPotencial; //[f=g+h, g, h]

    //Matriz de estados solucion
    std::vector<unsigned int> statesSolutionStates;
    std::vector< std::vector<double> > statesSolutionPotencial; //[f=g+h, g, h]


protected:
    //Costs
    virtual int gCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn);
    virtual int hCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn);


public:
    double maxCost;



public:

    AStar();
    ~AStar();

    int init();
    int clear();


public:
    int clearSearch();


public:

    int setGraphToSearch(DiscreteGraph *GraphToSearchIn);

protected:
    int searchInGraph(bool& pathFound);

    int planCreation();

public:
    int search(std::vector<double> nodeInitIn, std::vector<double> nodeFinIn, bool& pathFound);


public:
    int saveSolutionNodes(std::string nameNodesSol);


};


/////////////////////////////////////////
// Class AStarWithPotencialFieldMap
//
//   Description
//
/////////////////////////////////////////
class AStarWithPotencialFieldMap : public AStar
{

protected:
    PotencialFieldMap* MyPotFieldMap;


public:
    AStarWithPotencialFieldMap();
    const double h_sobrecoste;




protected:
    std::vector<unsigned int> simplifiedStatesSolution;



public:
    int setPotencialFieldMap(PotencialFieldMap* MyPotFieldMapIn);


protected:
    //Costs
    int gCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn);
    int hCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn);

public:
    int postCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn);

    //Search
public:
    int search(std::vector<double> nodeInitIn, std::vector<double> nodeFinIn, bool& pathFound);


protected:
    int postProcessSolution();

public:
    int saveSolutionNodes(std::string nameNodesSol, std::string nameNodesSolSimplif);


};







#endif
