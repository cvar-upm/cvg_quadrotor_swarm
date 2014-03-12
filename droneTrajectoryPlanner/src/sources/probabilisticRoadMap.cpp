//////////////////////////////////////////////////////
//  probabilisticRoadMap.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 23, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "probabilisticRoadMap.h"


using namespace std;



/////////////// DiscreteGraph //////////////
DiscreteGraph::DiscreteGraph()
{
    init();
    return;
}

DiscreteGraph::~DiscreteGraph()
{
    clear();
    return;
}

int DiscreteGraph::init()
{
    numNodes=0;
    nodesNeighborhood=0;
    dimension=0;

    isGeneratedBool=false;

    nodesList.clear();
    nodesRelation.clear();

    return 1;
}

int DiscreteGraph::clear()
{

    return 1;
}


//int DiscreteGraph::configure(int numNodesIn, int nodesNeighborhoodIn, int dimensionIn)
int DiscreteGraph::configure(int numNodesIn, int nodesNeighborhoodIn)
{
    //Set configuration
    numNodes=static_cast<unsigned int>(numNodesIn);
    nodesNeighborhood=static_cast<unsigned int>(nodesNeighborhoodIn);
    //dimension=(unsigned int)dimensionIn;

    //end
    return 1;
}



int DiscreteGraph::findNeighbordsNodes(int nodeI, std::vector<unsigned int> &neighborsNodeI)
{
    neighborsNodeI.clear();

    //if(!isGeneratedBool)
    //    return 0;

    for(unsigned int i=0;i<nodesRelation.size();i++)
    {
        if(nodesRelation[i][0]==nodeI)
            neighborsNodeI.push_back(nodesRelation[i][1]);
        else if(nodesRelation[i][1]==nodeI)
            neighborsNodeI.push_back(nodesRelation[i][0]);
    }

    return 1;

}




bool DiscreteGraph::areNeighbordsNodes(int nodeI, int nodeJ)
{
    //if(!isGeneratedBool)
    //    return false;

    for(unsigned int i=0;i<nodesRelation.size();i++)
    {
        if(nodesRelation[i][0]==nodeI && nodesRelation[i][1]==nodeJ)
            return true;

        if(nodesRelation[i][1]==nodeI && nodesRelation[i][0]==nodeJ)
            return true;
    }
    return false;

}

int DiscreteGraph::findNode(unsigned int &nodeIdOut, std::vector<double> nodeIn)
{
    if(!isGeneratedBool)
        return 0;

    for(unsigned int i=0;i<nodesList.size();i++)
    {
        if(nodeIn==nodesList[i])
        {
            nodeIdOut=i;
            return 1;
        }
    }
    return 0;
}


int DiscreteGraph::findNode(std::vector<double> &nodeOut, unsigned int nodeId)
{
    if(!isGeneratedBool)
        return 0;

    if(nodeId<nodesList.size())
    {
        nodeOut=nodesList[nodeId];
        return 1;
    }
    else
        return 0;

}


bool DiscreteGraph::isNodeInTheGraph(unsigned int nodeId)
{
    if(!isGeneratedBool)
        return false;

    if(nodeId<nodesList.size())
        return true;
    else
        return false;
}

bool DiscreteGraph::isNodeInTheGraph(std::vector<double> nodeIn)
{
    if(!isGeneratedBool)
        return false;

    for(unsigned int i=0;i<nodesList.size();i++)
    {
        if(nodeIn==nodesList[i])
        {
            return true;
        }
    }
    return false;
}


int DiscreteGraph::display()
{
    cout<<"Displaying Discrete Graph: Nodes"<<endl;
    for(unsigned int i=0; i<nodesList.size();i++)
    {
        cout<<"-Node "<<i<<": ";
        for(unsigned int j=0;j<dimension;j++)
            cout<<nodesList[i][j]<<" ";
        cout<<endl;
    }

    cout<<"Displaying Discrete Graph: Nodes relation"<<endl;
    for(unsigned int i=0; i<nodesRelation.size();i++)
    {
        cout<<"-Node "<<nodesRelation[i][0]<<" with node "<<nodesRelation[i][1]<<endl;
    }
    return 1;
}

int DiscreteGraph::save(std::string nameNodesList, std::string nameNodesRelation)
{
    ofstream myfile;

    ///Nodes list
    myfile.open(nameNodesList);
    for(unsigned int i=0; i<nodesList.size();i++)
    {
        for(unsigned int j=0;j<dimension;j++)
        {
            myfile<<nodesList[i][j];
            if(j==dimension-1)
                myfile<<endl;
            else
                myfile<<";";
        }
    }
    myfile.close();

    //Nodes relation
    myfile.open(nameNodesRelation);
    for(unsigned int i=0; i<nodesRelation.size();i++)
    {
        myfile<<nodesRelation[i][0]<<";"<<nodesRelation[i][1]<<endl;
    }
    myfile.close();

    return 1;
}


int DiscreteGraph::load(std::string nameNodesList, std::string nameNodesRelation)
{

    //Clear
    nodesList.clear();
    nodesRelation.clear();

    //Neig
    nodesNeighborhood=0;
    //Set num nodes!
    numNodes=0;
    //dimension
    dimension=0;

    //Variables
    ifstream fileToRead;

    string line;
    string token;


    ///////NodesList
    fileToRead.open(nameNodesList);

    if(!fileToRead.is_open())
        return 0;



    vector<double> nodeAux;
    float f;

    //Read
    while (getline(fileToRead, line))
    {
        nodeAux.clear();

        numNodes++;

        istringstream tokenizer(line);

        //we count the dimension
        if(dimension==0)
        {
            while(1)
            {
                token.clear();
                getline(tokenizer, token, ';');
                istringstream float_iss(token);

                if(token.size()==0)
                    break;
                else
                    dimension++;


                float_iss >> f;

                nodeAux.push_back(f);
            }

            nodesList.push_back(nodeAux);

            //cout<<"dimension="<<dimension<<endl;
        }
        else
        {
            for(unsigned int i=0;i<dimension;i++)
            {
                getline(tokenizer, token, ';');
                istringstream float_iss(token);

                float_iss >> f;

                nodeAux.push_back(f);
            }

            nodesList.push_back(nodeAux);
        }
    }

    fileToRead.close();




    ////////////NodesRelation
    fileToRead.open(nameNodesRelation);

    if(!fileToRead.is_open())
        return 0;


    vector<int> relationshipAux(2);


    //Read
    while (getline(fileToRead, line))
    {

        istringstream tokenizer(line);

        for(unsigned int i=0;i<2;i++)
        {
            getline(tokenizer, token, ';');
            istringstream int_iss(token);

            int_iss >> relationshipAux[i];

            //cout<<relationshipAux[i]<<endl;

            //nodeAux.push_back(f);
        }

        nodesRelation.push_back(relationshipAux);

    }

    fileToRead.close();



    //Count neigborhood!!
    int av=0;
    for(unsigned int i=0;i<numNodes;i++)
    {
        vector<unsigned int> relationNodeI;
        findNeighbordsNodes(i,relationNodeI);


        //for(int j=0;j<relationNodeI.size();j++)
            //cout<<"Node i="<<i<<" -> "<<relationNodeI.size()<<endl;
            av+=relationNodeI.size();

    }

    nodesNeighborhood=floor(static_cast<double>(av)/static_cast<double>(numNodes));

    //cout<<"av="<<(double)av/(double)numNodes<<endl;

    //cout<<nodesNeighborhood<<endl;


    if(numNodes>0 && dimension>0 && nodesNeighborhood>0)
        isGeneratedBool=true;




    return 1;
}





//////////////// PRM /////////////

ProbabilisticRoadMap::ProbabilisticRoadMap()
{
    init();
    return;
}


ProbabilisticRoadMap::~ProbabilisticRoadMap()
{
    clear();
    return;
}





int ProbabilisticRoadMap::init()
{
    //use current time as seed for random generator
    std::srand(std::time(0));

    return 1;
}


int ProbabilisticRoadMap::clear()
{

    return 1;
}






double ProbabilisticRoadMap::distanceBetweenNodes(std::vector<double> node1, std::vector<double> node2, int typeDist)
{
    double distance=0.0;
    switch(typeDist)
    {
    case 2:
        //Distancia euclidea
        for(unsigned int i=0;i<dimension;i++)
        {
            //cout<<node1[i]<<endl;
            //cout<<node2[i]<<endl;
            distance+=pow(node1[i]-node2[i],2);
        }

        distance=sqrt(distance);
        //cout<<"dist="<<distance<<endl;
        //getchar();
        break;
    }
    return distance;

}


int ProbabilisticRoadMap::generateFreePRM(WorldMap &mapIn)
{

    //return 0;

    //Check
    if(numNodes<1)
        return 0;
//cout<<"aqui\n";

    if(!mapIn.getDimension(dimension))
        return 0;

    //cout<<"dim for prm="<<dimension<<endl;

//cout<<"here"<<endl;
    if(dimension<1)
        return 0;
//cout<<"here2"<<endl;
//cout<<"aqui\n"<<nodesNeighborhood<<endl;
    if(nodesNeighborhood<1)
        return 0;
//cout<<"here3"<<endl;

//cout<<"aqui\n";

    ///// Random nodes generation
    //Clear list of nodes
    nodesList.clear();

    //std::srand(std::time(0)); //use current time as seed for random generator

    std::vector<double> newNode(dimension);

    std::vector<double> mapDimension;
    mapIn.getDimensions(mapDimension);

    std::vector<double> mapInitPoint;
    mapIn.getInitPoint(mapInitPoint);

    for(unsigned int i=0;i<numNodes;i++)
    {
        for(unsigned int j=0;j<dimension;j++)
        {
            //http://www.cplusplus.com/reference/cstdlib/rand/
            newNode[j] = ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) * mapDimension[j] + mapInitPoint[j];
            //std::cout<<i<<"; "<<j<<"; "<<newNode[j]<<std::endl;
        }
        nodesList.push_back(newNode);
    }



    ///// Conecting nodes


    //Distancias a nodos
    std::vector< std::vector<double> > distanciaNodos(numNodes);
    //init distances
    for(unsigned int i=0;i<numNodes;i++)
    {
        distanciaNodos[i].resize(numNodes);
        distanciaNodos[i][i]=0.0;
        /*
        for(int j=0;j<numNodes;j++)
        {
            distanciaNodos[i][j]=0.0;
        }
        */
    }
    //Calculate distances
    for(unsigned int i=0;i<numNodes;i++)
    {
        //distanciaNodos[i].resize(numNodes);
        for(unsigned int j=i+1;j<numNodes;j++)
        //for(int j=0;j<numNodes;j++)
        {
            distanciaNodos[i][j]=distanceBetweenNodes(nodesList[i],nodesList[j],2);
        }
    }

    //La matriz es simetrica. rellenamos
    for(unsigned int i=0;i<numNodes;i++)
    {
        for(unsigned int j=i+1;j<numNodes;j++)
        {
            distanciaNodos[j][i]=distanciaNodos[i][j];
        }
    }


    //Display
    /*
    for(int i=0;i<numNodes;i++)
    {
        for(int j=0;j<numNodes;j++)
        {
            std::cout<<distanciaNodos[i][j]<<" ";
        }
        std::cout<<std::endl;
        //getchar();
    }
    */


    //Nodes relation
    nodesRelation.clear();

    vector<int> newNeighbordNodesRelation(2);
    for(unsigned int i=0;i<numNodes;i++)
    {
        /*
        vector<double> aux(numNodes-(i+1));

        for(unsigned int j=0;j<aux.size();j++)
            aux[j]=distanciaNodos[i][j+(i+1)];
            */
        vector<double> nodeIDistances(numNodes);

        nodeIDistances=distanciaNodos[i];


        std::sort(nodeIDistances.begin(),nodeIDistances.end());


        /*
        cout<<"Distances sort:"<<endl;
        for(unsigned int j=0;j<nodeIDistances.size();j++)
            cout<<nodeIDistances[j]<<" ";
        cout<<endl;
        */


        for(unsigned int j=0;j<nodesNeighborhood;j++)
        {
            int nodeJ=0;

            nodeJ=std::find(distanciaNodos[i].begin(),distanciaNodos[i].end(),nodeIDistances[j+1])-distanciaNodos[i].begin();

            //cout<<"Node i="<<i<<" with node j="<<nodeJ<<endl;


            //We find this relation in nodesRelation. If doesnt exist, we add it.
            if(!areNeighbordsNodes(i,nodeJ))
            {
                newNeighbordNodesRelation[0]=i;
                newNeighbordNodesRelation[1]=nodeJ;

                nodesRelation.push_back(newNeighbordNodesRelation);

                //cout<<"Nodes="<<newNeighbordNodesRelation[0]<<" and "<<newNeighbordNodesRelation[1]<<endl;

            }

        }


    }



    /*
    cout<<"Node relation"<<endl;
    for(unsigned int i=0;i<nodesRelation.size();i++)
    {
        cout<<"node I="<<nodesRelation[i][0]<<", node J="<<nodesRelation[i][1]<<endl;
    }
    */


    /*
    cout<<endl<<"nodes neighbords"<<endl;
    for(unsigned int i=0;i<numNodes;i++)
    {
        vector<int> relationNodeI;
        findNeighbordsNodes(i,relationNodeI);


        for(int j=0;j<relationNodeI.size();j++)
            cout<<"Node i="<<i<<", node j="<<relationNodeI[j]<<endl;

    }
    */

    /*
    cout<<endl<<"nodes neighbords"<<endl;
    int av=0;
    for(unsigned int i=0;i<numNodes;i++)
    {
        vector<int> relationNodeI;
        findNeighbordsNodes(i,relationNodeI);


        //for(int j=0;j<relationNodeI.size();j++)
            cout<<"Node i="<<i<<" -> "<<relationNodeI.size()<<endl;
            av+=relationNodeI.size();

    }
    cout<<"av="<<(double)av/(double)numNodes<<endl;
    */

    isGeneratedBool=true;


    //end
    return 1;

}








int ProbabilisticRoadMap::addNode(std::vector<double> nodeIn)
{
    //Comprobaciones previas
    if(nodeIn.size()!=dimension)
        return 0;

    //Lista PRM generada
    if(!isGeneratedBool)
        return 0;


    //Comprobar que el nodo no está
    if(isNodeInTheGraph(nodeIn))
        return 1;

    //Nodo no conectado. conectamos

    //Add node to nodes lists
    numNodes++;
    nodesList.push_back(nodeIn);


    //Distance between nodes
    std::vector<double> distanciaNodos(numNodes);
    for(unsigned int i=0;i<nodesList.size();i++)
    {
        distanciaNodos[i]=distanceBetweenNodes(nodesList[i],nodesList[numNodes-1],2);
    }
//cout<<nodesList.size()<<endl;


    vector<int> newNeighbordNodesRelation(2);

    vector<double> nodeIDistances(numNodes);
    nodeIDistances=distanciaNodos;
    std::sort(nodeIDistances.begin(),nodeIDistances.end());


    for(unsigned int j=0;j<nodesNeighborhood;j++)
    {
        int nodeJ=0;

        nodeJ=std::find(distanciaNodos.begin(),distanciaNodos.end(),nodeIDistances[j+1])-distanciaNodos.begin();

        //cout<<"Node i="<<i<<" with node j="<<nodeJ<<endl;


        //We find this relation in nodesRelation. If doesnt exist, we add it.
        if(!areNeighbordsNodes(numNodes-1,nodeJ))
        {
            newNeighbordNodesRelation[0]=numNodes-1;
            newNeighbordNodesRelation[1]=nodeJ;

            nodesRelation.push_back(newNeighbordNodesRelation);

            //cout<<"Nodes="<<newNeighbordNodesRelation[0]<<" and "<<newNeighbordNodesRelation[1]<<endl;

        }

    }


    return 1;
}


int ProbabilisticRoadMap::addRandomNode(std::vector<double> initPointZone, std::vector<double> sizeZone)
{
    //Check
    if(dimension!=initPointZone.size() || dimension!=sizeZone.size())
        return 0;


    ///// Random node generation
    //std::srand(std::time(0)); //use current time as seed for random generator

    std::vector<double> newNode(dimension);


    //cout<<"New node:"<<endl;
    for(unsigned int j=0;j<dimension;j++)
    {
        //http://www.cplusplus.com/reference/cstdlib/rand/
        newNode[j] = ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) * sizeZone[j] + initPointZone[j];
        //std::cout<<newNode[j]<<"; ";
    }
    //cout<<endl;


    if(!addNode(newNode))
        return 0;


    return 1;
}



int ProbabilisticRoadMap::addRandomNodes(unsigned int numNodesAdd, std::vector<double> initPoint, std::vector<double> size)
{
    for(unsigned int i=0;i<numNodesAdd;i++)
    {
        if(!addRandomNode(initPoint,size))
            return 0;
    }
    return 1;
}


