//////////////////////////////////////////////////////
//  discreteSearch.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "discreteSearch.h"


using namespace std;

AStar::AStar() : maxCost(MAX_COST_A_STAR)
{
    std::default_random_engine generator;
    std::time_t result = std::time(NULL);
    generator.seed(result);
    std::uniform_real_distribution<double> distribution(1.0,1.5);

    double randnum=distribution(generator);

    maxCost/=randnum;

    cout<<"randnum="<<randnum<<"; maxCost="<<maxCost<<endl;

    init();
}

AStar::~AStar()
{   // TODO_P: What do you think about opening brackets here?
    return;
}

int AStar::init()
{
    return 1;
}

int AStar::clear()
{
    return 1;
}


int AStar::setGraphToSearch(DiscreteGraph *GraphToSearchIn)
{
    GraphToSearch=GraphToSearchIn;

    return 1;
}



int AStar::clearSearch()
{

    //Matriz de secuenciamiento de acciones
    secuenciamientoAcciones.clear(); //[identif nodo 1; identif nodo 2]

    //Matriz de almacenamiento de estados visitados
    statesVisitedStates.clear();
    statesVisitedPotencial.clear();

    //matriz de estados ya evaluados y ya expandidos
    statesClosedStates.clear();
    statesClosedPotencial.clear();

    //Matriz de estados abiertos. Todavia no evaluados
    statesOpenStates.clear();
    statesOpenPotencial.clear();


    //AllStates
    statesAllStates.clear();
    statesAllPotencial.clear();

    //Matriz de estados solucion
    statesSolutionStates.clear();
    statesSolutionPotencial.clear();

    return 1;
}


int AStar::gCost(double &costOut, std::vector<double> nodeI, std::vector<double> nodeJ)
{
    costOut=0.0;
    return 0;
}

int AStar::hCost(double &costOut, std::vector<double> nodeI, std::vector<double> nodeJ)
{
    costOut=0.0;
    return 0;
}



int AStar::searchInGraph(bool& pathFound)
{
    pathFound=false;

    //Clear previous search
    if(!clearSearch())
        return 0;


    //Usefull variables
    std::vector<unsigned int> secuencia(2);
    //Parent variables
    unsigned int parentId=0;
    vector<double> parentPotencial(3);
    //Child varianles
    unsigned int childId=0;
    vector<double> childPotencial(3);
    //Childs
    vector<unsigned int> childStatesIds;
    //Nodes for potencial
    vector<double> node1;
    vector<double> node2;



    //Start the search.
    //Put the init state in the list of open states
    statesOpenStates.push_back(nodeInitId);
    //Calculate potencials
    //g
    parentPotencial[1]=0.0; //g=0
    //h
    if(!GraphToSearch->findNode(node1,nodeFinId))
        return 0;
    if(!GraphToSearch->findNode(node2,nodeInitId))
        return 0;
    if(!hCost(parentPotencial[2],node1,node2))
        return 0;
    //f
    parentPotencial[0]=parentPotencial[1]+parentPotencial[2];
    statesOpenPotencial.push_back(parentPotencial);


    //Insertamos to all states
    for(unsigned int i=0;i<statesOpenStates.size();i++)
    {
        statesAllStates.push_back(statesOpenStates[i]);
        statesAllPotencial.push_back(statesOpenPotencial[i]);
    }

    //Search loop
    //bool endLoop=false;

    //cout<<"aqui"<<endl;

    //while(!endLoop && (statesVisitedStates.size()>0 || statesOpenStates.size()>0))
    while(statesVisitedStates.size()>0 || statesOpenStates.size()>0)
    {
        //cout<<"checking open states"<<endl;
        //Check the openStatesLists
        for(unsigned int i=0;i<statesOpenStates.size();i++)
        {
            //cout<<i<<"; "<<statesOpenStates.size()<<endl;
            //Put in the list of visited States
            statesVisitedStates.push_back(statesOpenStates[i]);
            statesVisitedPotencial.push_back(statesOpenPotencial[i]);
            //cout<<"venga!"<<endl;

            //cout<<statesOpenStates[i]<<endl;
            //cout<<nodeFinId<<endl;

            //Check we achieve the goal
            if(statesOpenStates[i]==nodeFinId)
            {
                //endLoop=true;
                pathFound=true;
                return 1;
            }
            //cout<<"here!"<<endl;
        }
        //cout<<"open states checked!"<<endl;
        //Clear the list of open states
        statesOpenStates.clear();
        statesOpenPotencial.clear();


        ///// Expandimos el estado
        //cout<<"expanding state"<<endl;
        //Selection of the parentState in the visited states list
        unsigned int parentIVisitedStates=0;
        double minPotencial=9e99;
        for(unsigned int i=0;i<statesVisitedPotencial.size();i++)
        {
            if(statesVisitedPotencial[i][0]<minPotencial)
            {
                parentIVisitedStates=i;
                minPotencial=statesVisitedPotencial[parentIVisitedStates][0]; //f
            }
        }

        //Check minimun cost
        //cout<<"checking cost"<<endl;
        if(minPotencial>maxCost)
        {
            pathFound=false;
            return 1;
        }

        //if(statesVisitedPotencial[parentIVisitedStates][1]>maxCost)
        //    return 0;
        //Habria que hacer lazy-PRM y generar nuevos nodos en las zonas de muy alto potencial!!


        //Parent
        parentId=statesVisitedStates[parentIVisitedStates];
        parentPotencial=statesVisitedPotencial[parentIVisitedStates];


        //cout<<"Parent node="<<parentId<<endl;

        //Eliminamos el estado padre de la lista de visited states
        statesVisitedStates.erase(statesVisitedStates.begin()+parentIVisitedStates);
        statesVisitedPotencial.erase(statesVisitedPotencial.begin()+parentIVisitedStates);


        //Añadimos el estado padre a la lista de closed states
        statesClosedStates.push_back(parentId);
        statesClosedPotencial.push_back(parentPotencial);



        //cout<<parentIVisitedStates<<"; "<<parentId<<endl;
        //getchar();
        //GraphToSearch.findNode(parentState,parentId);




        //Buscamos vecinos (hijos) del estado padre -> Expandimos
        if(!GraphToSearch->findNeighbordsNodes(parentId,childStatesIds))
            return 0;


        //cout<<"Padre:"<<parentId<<endl;


        ///// Hacemos crecer a los hijos!
        for(unsigned int numChildI=0;numChildI<childStatesIds.size();numChildI++)
        {
            //cout<<"hola bucle!!!!! "<<childStatesIds.size()<<endl;

            childId=childStatesIds[numChildI];

            //cout<<"-Hijo="<<childStatesIds[numChildI]<<endl;

            //Comprobamos si el estado ya está visitado. Está en la lista de estados visitados
            bool alreadyVisited=false;
            unsigned int nodeIVisitedStates=0;
            for(unsigned int i=0;i<statesVisitedStates.size();i++)
            {
                if(statesVisitedStates[i]==childId)
                {
                    nodeIVisitedStates=i;
                    alreadyVisited=true;
                    //cout<<" -Estado visitado: "<<statesVisitedStates[i]<<endl;
                    break;
                }
            }

            //Comprobamos si el estado ya está en la lista de estados cerrados
            bool alreadyClosed=false;
            for(unsigned int i=0;i<statesClosedStates.size();i++)
            {
                if(statesClosedStates[i]==childStatesIds[numChildI])
                {
                    alreadyClosed=true;
                    //cout<<" -Estado cerrado: "<<statesClosedStates[i]<<endl;
                    break;
                }
            }

            //Calculamos el nuevo potencial

            //cout<<"   .Potencial"<<endl;

            //g
            if(!GraphToSearch->findNode(node1,parentId))
                return 0;
            if(!GraphToSearch->findNode(node2,childId))
                return 0;
            //cout<<"ei"<<endl;
            if(!gCost(childPotencial[1],node1,node2))
                return 0;
            childPotencial[1]+=parentPotencial[1];

            //cout<<"ei"<<endl;

            //h
            if(!GraphToSearch->findNode(node1,childId))
                return 0;
            if(!GraphToSearch->findNode(node2,nodeFinId))
                return 0;
            if(!hCost(childPotencial[2],node1,node2))
                return 0;

            //f
            childPotencial[0]=childPotencial[1]+childPotencial[2];



            //Si el estado no ha sido visitado nunca. Es nuevo
            if(!alreadyVisited && !alreadyClosed)
            {
                //cout<<"  estado nuevo"<<endl;

                //Añadimos a la lista de estados abiertos
                statesOpenStates.push_back(childId);
                statesOpenPotencial.push_back(childPotencial);

                //Añadimos a la lista de all states
                statesAllStates.push_back(childId);
                statesAllPotencial.push_back(childPotencial);

                //Añadimos la secuencia
                secuencia[0]=parentId;
                secuencia[1]=childId;
                secuenciamientoAcciones.push_back(secuencia);

                //cout<<"  yiee!!"<<endl;
                //cout<<"hola!!"<<endl;

            }
            else if(alreadyVisited)
            {
            //El estado ya habia sido visitado. Y está en la lista de visitados

                //cout<<"  estado ya visitado!!"<<endl;

                //Comparamos el potencial para alcanzar ese nodo
                if(childPotencial[1]<statesVisitedPotencial[nodeIVisitedStates][1])
                {
                    //Cambiamos el potencial en la lista de visitados
                    statesVisitedPotencial[nodeIVisitedStates]=childPotencial;

                    //Cambiamos el potencial en la lista de all states
                    for(unsigned int i=0;i<statesAllStates.size();i++)
                    {
                        if(statesAllStates[i]==childId)
                        {
                            statesAllPotencial[i]=childPotencial;
                            break;
                        }
                    }

                    //Cambiamos el secuenciamiento
                    for(unsigned int i=0;i<secuenciamientoAcciones.size();i++)
                    {
                        if(secuenciamientoAcciones[i][1]==childId)
                        {
                            secuenciamientoAcciones[i][0]=parentId;
                        }
                    }

                }


            }
            else
            {
            //El estado está en la lista de cerrados. No pasa nada
            }
            //cout<<"dime"<<endl;
            //cout<<"aqui"<<endl;
        }
        //cout<<"fin bucle"<<endl;
    }
    //cout<<"problema!"<<endl;

    pathFound=false;
    return 0;
}


int AStar::planCreation()
{

    //Init
    unsigned int idPadre=nodeFinId;
    unsigned int idHijo=nodeFinId;


    //statesSolutionStates.clear();
    //statesSolutionPotencial.clear();


    statesSolutionStates.push_back(nodeFinId);

    while(idPadre!=nodeInitId)
    {
        //Search parent of child
        for(unsigned int i=0;i<secuenciamientoAcciones.size();i++)
        {
            if(secuenciamientoAcciones[i][1]==idHijo)
            {
                idPadre=secuenciamientoAcciones[i][0];
                break;
            }
        }
        //Add parent to the list
        statesSolutionStates.push_back(idPadre);

        //Update for next loop
        idHijo=idPadre;
    }

    std::reverse(statesSolutionStates.begin(),statesSolutionStates.end());

    //Añadimos el potencial
    for(unsigned int i=0;i<statesSolutionStates.size();i++)
    {
        for(unsigned int j=0;j<statesAllStates.size();j++)
        {
            if(statesSolutionStates[i]==statesAllStates[j])
            {
                statesSolutionPotencial.push_back(statesAllPotencial[j]);
                break;
            }
        }
    }

    return 1;
}



int AStar::search(std::vector<double> nodeInitIn, std::vector<double> nodeFinIn, bool& pathFound)
{
    pathFound=false;

    //Set nodes init and fin
    nodeInit=nodeInitIn;
    nodeFin=nodeFinIn;

    //Add the nodes init and end
    if(!GraphToSearch->findNode(nodeInitId,nodeInit))
        return 0;
    if(!GraphToSearch->findNode(nodeFinId,nodeFin))
        return 0;


    //cout<<"searching in graph!"<<endl;

    //Search in the graph
    if(!searchInGraph(pathFound))
        return 0;


    //cout<<"creating plan!"<<endl;

    //Creation of the plan
    if(pathFound)
    {
        if(!planCreation())
            return 0;
    }

    //cout<<"plan created!"<<endl;


    //Solution
    vector<double> nodeAux;
    solutionNodes.clear();
    for(unsigned int i=0;i<statesSolutionStates.size();i++)
    {
        GraphToSearch->findNode(nodeAux,statesSolutionStates[i]);
        solutionNodes.push_back(nodeAux);
    }




    return 1;
}

int AStar::getSolution(std::vector< std::vector<double> > &solutionNodesOut)
{
    solutionNodesOut=solutionNodes;

    return 1;
}


int AStar::saveSolutionNodes(std::string nameNodesSol)
{
    ofstream myfile;

    myfile.open(nameNodesSol);

    /*for(unsigned int i=0;i<statesSolutionStates.size();i++)
    {
        //cout<<"-node: "<<statesSolutionStates[i]<<endl;
        myfile<<statesSolutionStates[i]<<endl;
    }
    */



    std::vector<double> nodeAux;

    for(unsigned int i=0;i<statesSolutionStates.size();i++)
    {

        GraphToSearch->findNode(nodeAux,statesSolutionStates[i]);

        for(unsigned int j=0;j<nodeAux.size();j++)
        {
            myfile<<nodeAux[j];
            if(j==nodeAux.size()-1)
                myfile<<endl;
            else
                myfile<<";";
        }

        //cout<<"-node: "<<statesSolutionStates[i]<<endl;
        //myfile<<simplifiedStatesSolution[i]<<endl;
    }

    myfile.close();

    return 1;

}














//////////////////////// AStarWithPotencialFieldMap /////////////////////////

AStarWithPotencialFieldMap::AStarWithPotencialFieldMap() : h_sobrecoste(H_OVERCOST) {
}

int AStarWithPotencialFieldMap::gCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn)
{
    //cout<<"gcost"<<endl;

    return MyPotFieldMap->calculateDistPotMap(costOut,nodeInitIn,nodeFinIn,true,true);

    //return 1;
}


int AStarWithPotencialFieldMap::hCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn)
{
    int ok=MyPotFieldMap->calculateDistPotMap(costOut,nodeInitIn,nodeFinIn,false,true);
    //int ok=MyPotFieldMap->calculateDistPotMap(costOut,nodeInitIn,nodeFinIn,true,true);

    costOut*=h_sobrecoste;

    return ok;
}

int AStarWithPotencialFieldMap::postCost(double &costOut, std::vector<double> nodeInitIn, std::vector<double> nodeFinIn)
{
    return MyPotFieldMap->calculateDistPotMap(costOut,nodeInitIn,nodeFinIn,true,false);

    //return 1;
}


int AStarWithPotencialFieldMap::setPotencialFieldMap(PotencialFieldMap* MyPotFieldMapIn)
{
    MyPotFieldMap=MyPotFieldMapIn;
    return 1;
}


int AStarWithPotencialFieldMap::postProcessSolution()
{
    //Init
    simplifiedStatesSolution.clear();

    //Previuos checks
    if(statesSolutionStates.size()<2)
        return 0;

    vector<double> costVector;
    double costI;

    costVector.push_back(0.0);

    vector<double> node1;
    unsigned int node1Id;
    vector<double> node2;
    unsigned int node2Id;

    node1Id=statesSolutionStates[0];
    if(!GraphToSearch->findNode(node1,node1Id))
        return 0;

    //Coste para alcanzar el estado I desde el estado I-1
    for(unsigned int i=1;i<statesSolutionStates.size();i++)
    {
        node2Id=statesSolutionStates[i];
        if(!GraphToSearch->findNode(node2,node2Id))
            return 0;

        if(!postCost(costI,node1,node2))
            return 0;

        costVector.push_back(costI);

        //Update for the next step
        node1=node2;

    }

    /*
    cout<<"cost vector="<<endl;
    for(unsigned int i=0;i<costVector.size();i++)
        cout<<costVector[i]<<"; ";
    cout<<endl;
    */

    //cout<<"aqui"<<endl;

    //simplified solution
    //vector<unsigned int> simplifiedSolution;

    //Ultimo nodo!
    node1Id=statesSolutionStates[statesSolutionStates.size()-1];
    if(!GraphToSearch->findNode(node1,node1Id))
        return 0;

    unsigned int lastNodeToSearch=statesSolutionStates.size()-1;



    simplifiedStatesSolution.push_back(node1Id);

    double simplifiedCost=0.0;
    double nonSimplifiedCost=0.0;


    while(node1Id!=statesSolutionStates[0])
    {
        //cout<<"lastNodeToSearch="<<lastNodeToSearch<<endl;
        for(unsigned int i=0;i<lastNodeToSearch;i++)
        {
            //Set node
            node2Id=statesSolutionStates[i];
            if(!GraphToSearch->findNode(node2,node2Id))
                return 0;

            //Coste simplificado
            if(!postCost(simplifiedCost,node1,node2))
                return 0;

            //Coste no simplificado
            nonSimplifiedCost=0.0;
            for(unsigned int j=i;j<=lastNodeToSearch;j++)
                nonSimplifiedCost+=costVector[j];

            //cout<<i<<endl;
            //cout<<"simplifiedCost="<<simplifiedCost<<endl;
            //cout<<"nonSimplifiedCost="<<nonSimplifiedCost<<endl;
            if(simplifiedCost<=nonSimplifiedCost)
            {
                lastNodeToSearch=i;
                node1=node2;
                node1Id=node2Id;
                simplifiedStatesSolution.push_back(node2Id);
                break;
            }

            //else
            //{
                /*
                for(unsigned int j=lastNodeToSearch-1;j>0;j--)
                {
                    simplifiedStatesSolution.push_back(statesSolutionStates[j]);
                }
                */
                //simplifiedStatesSolution.push_back(node2Id);

                //lastNodeToSearch=i;
                //node1=node2;
                //node1Id=node2Id;

                //node1Id=statesSolutionStates[0];

                //break;
            //}


        }
        //cout<<"simplifiedCost="<<simplifiedCost<<endl;
        //cout<<"nonSimplifiedCost="<<nonSimplifiedCost<<endl;

        if(simplifiedCost>nonSimplifiedCost)
        {
            simplifiedStatesSolution.push_back(node2Id);
            lastNodeToSearch--;
            node1=node2;
            node1Id=node2Id;
        }

        //Comprobamos que hemos alcanzado el nodo inicial
        //cout<<"node1Id="<<node1Id<<endl;
        //cout<<"statesSolutionStates[0]="<<statesSolutionStates[0]<<endl;
        /*
        if(node1Id==statesSolutionStates[0])
        {
            break;
        }
        */

    }

    //statesSolutionStates.clear();
    //statesSolutionPotencial.clear();

    //statesSolutionStates=simplifiedSolution;

    //std::reverse(statesSolutionStates.begin(),statesSolutionStates.end());

    std::reverse(simplifiedStatesSolution.begin(),simplifiedStatesSolution.end());


    return 1;
}



int AStarWithPotencialFieldMap::search(std::vector<double> nodeInitIn, std::vector<double> nodeFinIn, bool& pathFound)
{
    pathFound=false;

    //Discrete search
    if(!AStar::search(nodeInitIn,nodeFinIn,pathFound))
        return 0;

    if(!pathFound)
    {
        simplifiedStatesSolution.clear();
        return 1;
    }

    //Admisible solution. Already done in the A Star, but por si aca!
    if(statesSolutionPotencial[statesSolutionStates.size()-1][1]>maxCost)
    {
        statesSolutionStates.clear();
        statesSolutionPotencial.clear();
        return 0;
    }

    //cout<<"Postprocessing solution"<<endl;

    //Postprocess solution
    if(!postProcessSolution())
        return 0;

    //cout<<"solution postprocessed!"<<endl;


    //Solution override
    vector<double> nodeAux;
    solutionNodes.clear();
    for(unsigned int i=0;i<simplifiedStatesSolution.size();i++)
    {
        GraphToSearch->findNode(nodeAux,simplifiedStatesSolution[i]);
        solutionNodes.push_back(nodeAux);
    }


    return 1;
}


int AStarWithPotencialFieldMap::saveSolutionNodes(std::string nameNodesSol, std::string nameNodesSolSimplif)
{
    AStar::saveSolutionNodes(nameNodesSol);


    //Simplified solution nodes
    ofstream myfile;

    myfile.open(nameNodesSolSimplif);

    std::vector<double> nodeAux;

    for(unsigned int i=0;i<simplifiedStatesSolution.size();i++)
    {

        GraphToSearch->findNode(nodeAux,simplifiedStatesSolution[i]);

        for(unsigned int j=0;j<nodeAux.size();j++)
        {
            myfile<<nodeAux[j];
            if(j==nodeAux.size()-1)
                myfile<<endl;
            else
                myfile<<";";
        }

        //cout<<"-node: "<<statesSolutionStates[i]<<endl;
        //myfile<<simplifiedStatesSolution[i]<<endl;
    }

    myfile.close();

    return 1;

}
