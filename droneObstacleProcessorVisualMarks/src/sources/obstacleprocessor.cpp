

#include "obstacleprocessor.h"
#include "pose.h"
#include "rectangle.h"
#include "circle.h"
#include <assert.h>
#include <cstdlib>

#include "pugixml.hpp"

ObstacleProcessor::ObstacleProcessor(void)
{

}

ObstacleProcessor::~ObstacleProcessor(void)
{
    for (unsigned int i=0;i<detectedObstacles.size();i++)
        delete detectedObstacles[i];
    detectedObstacles.clear();

}



void ObstacleProcessor::initialization(std::string paramsFileName)
{
	//FIXME, check!!
	/*wall_width = 0.2; //not given
	wall_length = 8.0;
	
	large_window_length = 1.5;
	small_window_length = 0.8;
	known_pole_rad = 0.20; //not given
	unknown_pole_rad = 0.20 + 0.1; //approx 40 cm + 10 cm security margin
	
	num_landmarks_wind = 8;*/
	
    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(paramsFileName);
    pugi::xml_parse_result result = doc.load(nameFile);

   if (!result) //FIXME, pass as argument
   {
   	cout << "Xml file missing " << endl;
   	assert(0);
    
   }
	
    wall_width = atof(doc.child("map").child_value("wall_width"));
    wall_length = atof(doc.child("map").child_value("wall_length"));
    //known_pole_rad = atof(doc.child("main").child("params").child_value("known_pole_rad"));
    //unknown_pole_rad = atof(doc.child("main").child("params").child_value("unknown_pole_rad"));
    large_window_length = atof(doc.child("map").child_value("large_window_length"));
    small_window_length = atof(doc.child("map").child_value("small_window_length"));

    aruco_width = atof(doc.child("map").child_value("aruco_width"));
	
    num_landmarks_wind = atoi(doc.child("map").child_value("num_landmarks_wind"));
    windows_distribution_option = atoi(doc.child("map").child_value("windows_distribution_option"));
	
    cout << "windows distribution option " << windows_distribution_option << endl;


    pugi::xml_node obstacles = doc.child("map").child("obstaclesInMap");

    //Obstacles!
    int idObstacle;
    set<int> arucosId;
    //Polos
    vector<double> obstRadius(2);


    //Reading poles!!
    for(pugi::xml_node pole = obstacles.child("pole"); pole; pole = pole.next_sibling("pole"))
    {

        std::string readingValue;


        //Id obstacle
        readingValue=pole.child_value("id");
        istringstream convertId(readingValue);
        convertId>>idObstacle;



        //Radius
        readingValue=pole.child_value("radius");
        istringstream convertSize(readingValue);
        for(unsigned int i=0;i<2;i++)
        {
            convertSize>>obstRadius[i];
        }

        //Arucos id
        pugi::xml_node aruco = pole.child("aruco");
        arucosId.clear();
        for(pugi::xml_node id = aruco.child("id"); id; id = id.next_sibling("id"))
        {
            int arucoId=-1;
            arucoId=id.attribute("aruco_id").as_int();

            //readingValue=id.value();
            //cout<<readingValue<<endl;
            //istringstream convertSize(readingValue);
            //convertSize>>arucoId;

            //cout<<arucoId<<endl;

            arucosId.insert(arucoId);
        }



        //Add obstacle
        Circle c;
        c.arucoIds = arucosId;
        c.radius = obstRadius[0];
        c.id=idObstacle;


        cout<<"add pole id="<<idObstacle<<"; radius="<<obstRadius[0]<<"; arucos id="<<endl;

        for(set<int>::iterator it = arucosId.begin();it!=arucosId.end();it++)
            cout << (*it) << " ";
        cout<<endl;

        existingPoles.push_back(c);

        //



    }
    cout << "size "  << existingPoles.size() << endl;

}


/*vector<Obstacle2D> ObstacleProcessor::getObstacles(vector<Landmark3D> v)
{

	obstacles.clear();
	
	Landmark3DMap lm_map;
	for (int i=0; i<v.size(); i++)
	{
		Landmark3D lm = v[i];
		int key = lm.id;
		lm_map[key]=lm;
	}
	
	//small window
	Landmark3DMap::iterator large_window_1  = lm_map.find(1);
	Landmark3DMap::iterator large_window_2  = lm_map.find(2);
	Landmark3DMap::iterator large_window_3  = lm_map.find(3);
	Landmark3DMap::iterator large_window_4  = lm_map.find(4);
	
	//large window
	Landmark3DMap::iterator small_window_1  = lm_map.find(5);
	Landmark3DMap::iterator small_window_2  = lm_map.find(6);
	Landmark3DMap::iterator small_window_3  = lm_map.find(7);
	Landmark3DMap::iterator small_window_4  = lm_map.find(8);
	
	if (large_window_1 != lm_map.end() && large_window_2 != lm_map.end())
	{
		double length1 = ((*large_window_1).second.x + (*large_window_2).second.x)/2;
		double width1 = wall_width;
		Pose p(length1/2,wall_width/2,0);
		Rectangle r1(p,length1,width1);
		obstacles.push_back(r1);
		
		if (large_window_3 != lm_map.end() && large_window_4 != lm_map.end() && small_window_1 != lm_map.end() && small_window_2 != lm_map.end())
		{
			double length2 = ((*small_window_1).second.x + (*small_window_2).second.x)/2 - ((*large_window_4).second.x + (*large_window_3).second.x)/2;
			double width2 = wall_width;
			Pose p2(length1 + large_window_length + length2/2,wall_width/2,0);
			Rectangle r2(p2,length2,width2);
			obstacles.push_back(r2);
			
			if (small_window_3 != lm_map.end() && small_window_4 != lm_map.end())
			{
				double length3 = wall_length - (length1 + large_window_length + length2 + small_window_length);
				double width3 = wall_width;
				Pose p3(length1 + large_window_length + length2 + small_window_length + length3/2,wall_width/2,0);
				Rectangle r3(p3,length3,width3);
				obstacles.push_back(r3);
	
			}
			
			
	
		}
	
	}

	
	////////////////////////////////////////////////////////////////
	// first known pole
	Landmark3DMap::iterator known_pole1_1  = lm_map.find(9);
	Landmark3DMap::iterator known_pole1_2  = lm_map.find(10);
	Landmark3DMap::iterator known_pole1_3  = lm_map.find(11);
	Landmark3DMap::iterator known_pole1_4  = lm_map.find(12);
	
	if (known_pole1_1 != lm_map.end() && known_pole1_2 != lm_map.end() && known_pole1_3 != lm_map.end() && known_pole1_4 != lm_map.end())
	{
		double x1 = ((*known_pole1_1).second.x + (*known_pole1_2).second.x + (*known_pole1_3).second.x + (*known_pole1_4).second.x)/4;
		double y1 = ((*known_pole1_1).second.y + (*known_pole1_2).second.y + (*known_pole1_3).second.y + (*known_pole1_4).second.y)/4;
		
		Circle known_pole_1(x1,y1,known_pole_rad);
		obstacles.push_back(known_pole_1);
	
	}
	
	// second known pole
	Landmark3DMap::iterator known_pole2_1  = lm_map.find(13);
	Landmark3DMap::iterator known_pole2_2  = lm_map.find(14);
	Landmark3DMap::iterator known_pole2_3  = lm_map.find(15);
	Landmark3DMap::iterator known_pole2_4  = lm_map.find(16);
	
	if (known_pole2_1 != lm_map.end() && known_pole2_2 != lm_map.end() && known_pole2_3 != lm_map.end() && known_pole2_4 != lm_map.end())
	{
		double x2 = ((*known_pole2_1).second.x + (*known_pole2_2).second.x + (*known_pole2_3).second.x + (*known_pole2_4).second.x)/4;
		double y2 = ((*known_pole2_1).second.y + (*known_pole2_2).second.y + (*known_pole2_3).second.y + (*known_pole2_4).second.y)/4;
		
		Circle known_pole_2(x2,y2,known_pole_rad);
		obstacles.push_back(known_pole_2);
	
	}
	
	// third known pole
	Landmark3DMap::iterator known_pole3_1  = lm_map.find(17);
	Landmark3DMap::iterator known_pole3_2  = lm_map.find(18);
	Landmark3DMap::iterator known_pole3_3  = lm_map.find(19);
	Landmark3DMap::iterator known_pole3_4  = lm_map.find(20);
	
	if (known_pole3_1 != lm_map.end() && known_pole3_2 != lm_map.end() && known_pole3_3 != lm_map.end() && known_pole3_4 != lm_map.end())
	{
		double x3 = ((*known_pole3_1).second.x + (*known_pole3_2).second.x + (*known_pole3_3).second.x + (*known_pole3_4).second.x)/4;
		double y3 = ((*known_pole3_1).second.y + (*known_pole3_2).second.y + (*known_pole3_3).second.y + (*known_pole3_4).second.y)/4;
		
		Circle known_pole_3(x3,y3,known_pole_rad);
		obstacles.push_back(known_pole_3);
	
	}
	
	
	
	// fourth known pole
	Landmark3DMap::iterator known_pole4_1  = lm_map.find(21);
	Landmark3DMap::iterator known_pole4_2  = lm_map.find(22);
	Landmark3DMap::iterator known_pole4_3  = lm_map.find(23);
	Landmark3DMap::iterator known_pole4_4  = lm_map.find(24);
	
	if (known_pole4_1 != lm_map.end() && known_pole4_2 != lm_map.end() && known_pole4_3 != lm_map.end() && known_pole4_4 != lm_map.end())
	{
		double x4 = ((*known_pole4_1).second.x + (*known_pole4_2).second.x + (*known_pole4_3).second.x + (*known_pole4_4).second.x)/4;
		double y4 = ((*known_pole4_1).second.y + (*known_pole4_2).second.y + (*known_pole4_3).second.y + (*known_pole4_4).second.y)/4;
		
		Circle known_pole_4(x4,y4,known_pole_rad);
		obstacles.push_back(known_pole_4);
	
	}
	
	
	////////////////////////////////////////////////////////////////
	// first unknown pole
	Landmark3DMap::iterator unknown_pole1_1  = lm_map.find(25);
	Landmark3DMap::iterator unknown_pole1_2  = lm_map.find(26);
	Landmark3DMap::iterator unknown_pole1_3  = lm_map.find(27);
	Landmark3DMap::iterator unknown_pole1_4  = lm_map.find(28);
	
	int visibility_case1;	
	
	//FIXME, include visibility case with a single landmark detection!!
		
	if (unknown_pole1_1 != lm_map.end() && unknown_pole1_2 != lm_map.end())
		visibility_case1 = 1; // 1 & 2
		
	if (unknown_pole1_1 != lm_map.end() && unknown_pole1_3 != lm_map.end())
		visibility_case1 = 2; // 1 & 3
		
	if (unknown_pole1_1 != lm_map.end() && unknown_pole1_4 != lm_map.end())
		visibility_case1 = 3; // 1 & 4	
		
	if (unknown_pole1_2 != lm_map.end() && unknown_pole1_3 != lm_map.end())
		visibility_case1 = 4; // 2 & 3	
	
	if (unknown_pole1_1 != lm_map.end() && unknown_pole1_3 != lm_map.end())
		visibility_case1 = 5; // 2 & 4
		
	if (unknown_pole1_1 != lm_map.end() && unknown_pole1_3 != lm_map.end())
		visibility_case1 = 6; // 3 & 4
		
	if (unknown_pole1_1 != lm_map.end() && unknown_pole1_2 != lm_map.end() && unknown_pole1_3 != lm_map.end() && unknown_pole1_4 != lm_map.end())
		visibility_case1 = 7; // all four landmarks detected
		
	
	double x1_unknown, y1_unknown;	
	if (visibility_case1 == 1) // 1 & 2
	{
		Vector2D dir2(cos((*unknown_pole1_2).second.roll),sin((*unknown_pole1_2).second.roll));
		Vector2D c1 = Vector2D((*unknown_pole1_1).second.x, (*unknown_pole1_1).second.y) + dir2*unknown_pole_rad ;
		
		Circle unknown_pole_1(c1.x,c1.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	if (visibility_case1 == 2) //1 & 3
	{
		x1_unknown = ((*unknown_pole1_1).second.x + (*unknown_pole1_3).second.x)/2;
		y1_unknown = ((*unknown_pole1_1).second.y + (*unknown_pole1_3).second.y)/2;
		
		Circle unknown_pole_1(x1_unknown,y1_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	if (visibility_case1 == 3) // 1 & 4
	{
		Vector2D dir4(cos((*unknown_pole1_4).second.roll),sin((*unknown_pole1_4).second.roll));
		Vector2D c1 = Vector2D((*unknown_pole1_1).second.x, (*unknown_pole1_1).second.y) - dir4*unknown_pole_rad ;
		
		Circle unknown_pole_1(c1.x,c1.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	if (visibility_case1 == 4) // 2 & 3
	{
		Vector2D dir3(cos((*unknown_pole1_3).second.roll),sin((*unknown_pole1_3).second.roll));
		Vector2D c1 = Vector2D((*unknown_pole1_2).second.x, (*unknown_pole1_2).second.y) + dir3*unknown_pole_rad ;
		
		Circle unknown_pole_1(c1.x,c1.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	if (visibility_case1 == 5) // 2 & 4
	{
		x1_unknown = ((*unknown_pole1_2).second.x + (*unknown_pole1_4).second.x)/2;
		y1_unknown = ((*unknown_pole1_2).second.y + (*unknown_pole1_4).second.y)/2;
		
		Circle unknown_pole_1(x1_unknown,y1_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	if (visibility_case1 == 6) // 3 & 4
	{
		Vector2D dir4(cos((*unknown_pole1_4).second.roll),sin((*unknown_pole1_4).second.roll));
		Vector2D c1 = Vector2D((*unknown_pole1_3).second.x, (*unknown_pole1_3).second.y) + dir4*unknown_pole_rad ;
		
		Circle unknown_pole_1(c1.x,c1.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	
	if (visibility_case1 == 7) // 1, 2, 3 & 4
	{
		x1_unknown = ((*unknown_pole1_1).second.x + (*unknown_pole1_2).second.x + (*unknown_pole1_3).second.x + (*unknown_pole1_4).second.x)/4;
		y1_unknown = ((*unknown_pole1_1).second.y + (*unknown_pole1_2).second.y + (*unknown_pole1_3).second.y + (*unknown_pole1_4).second.y)/4;
		
		Circle unknown_pole_1(x1_unknown,y1_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_1);
	}
	
	////////////////////////////////////////////////////////////////
	// second unknown pole
	Landmark3DMap::iterator unknown_pole2_1  = lm_map.find(29);
	Landmark3DMap::iterator unknown_pole2_2  = lm_map.find(30);
	Landmark3DMap::iterator unknown_pole2_3  = lm_map.find(31);
	Landmark3DMap::iterator unknown_pole2_4  = lm_map.find(32);
	
	int visibility_case2;	
		
	if (unknown_pole2_1 != lm_map.end() && unknown_pole2_2 != lm_map.end())
		visibility_case2 = 1; // 1 & 2
		
	if (unknown_pole2_1 != lm_map.end() && unknown_pole2_3 != lm_map.end())
		visibility_case2 = 2; // 1 & 3
		
	if (unknown_pole2_1 != lm_map.end() && unknown_pole2_4 != lm_map.end())
		visibility_case2 = 3; // 1 & 4	
		
	if (unknown_pole2_2 != lm_map.end() && unknown_pole2_3 != lm_map.end())
		visibility_case2 = 4; // 2 & 3	
	
	if (unknown_pole2_1 != lm_map.end() && unknown_pole2_3 != lm_map.end())
		visibility_case2 = 5; // 2 & 4
		
	if (unknown_pole2_1 != lm_map.end() && unknown_pole2_3 != lm_map.end())
		visibility_case2 = 6; // 3 & 4
		
	if (unknown_pole2_1 != lm_map.end() && unknown_pole2_2 != lm_map.end() && unknown_pole2_3 != lm_map.end() && unknown_pole2_4 != lm_map.end())
		visibility_case2 = 7; // all four landmarks detected
		
	
	double x2_unknown, y2_unknown;	
	if (visibility_case2 == 1) // 1 & 2
	{
		Vector2D dir2(cos((*unknown_pole1_2).second.roll),sin((*unknown_pole1_2).second.roll));
		Vector2D c2 = Vector2D((*unknown_pole1_1).second.x, (*unknown_pole1_1).second.y) + dir2*unknown_pole_rad ;
		
		Circle unknown_pole_2(c2.x,c2.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	if (visibility_case2 == 2) //1 & 3
	{
		x2_unknown = ((*unknown_pole2_1).second.x + (*unknown_pole2_3).second.x)/2;
		y2_unknown = ((*unknown_pole2_1).second.y + (*unknown_pole2_3).second.y)/2;
		
		Circle unknown_pole_2(x2_unknown,y2_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	if (visibility_case2 == 3) // 1 & 4
	{
		Vector2D dir4(cos((*unknown_pole2_4).second.roll),sin((*unknown_pole2_4).second.roll));
		Vector2D c2 = Vector2D((*unknown_pole2_1).second.x, (*unknown_pole2_1).second.y) - dir4*unknown_pole_rad ;
		
		Circle unknown_pole_2(c2.x,c2.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	if (visibility_case2 == 4) // 2 & 3
	{
		Vector2D dir3(cos((*unknown_pole2_3).second.roll),sin((*unknown_pole2_3).second.roll));
		Vector2D c2 = Vector2D((*unknown_pole2_2).second.x, (*unknown_pole2_2).second.y) + dir3*unknown_pole_rad ;
		
		Circle unknown_pole_2(c2.x,c2.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	if (visibility_case2 == 5) // 2 & 4
	{
		x2_unknown = ((*unknown_pole2_2).second.x + (*unknown_pole2_4).second.x)/2;
		y2_unknown = ((*unknown_pole2_2).second.y + (*unknown_pole2_4).second.y)/2;
		
		Circle unknown_pole_2(x2_unknown,y2_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	if (visibility_case2 == 6) // 3 & 4
	{
		Vector2D dir4(cos((*unknown_pole2_4).second.roll),sin((*unknown_pole2_4).second.roll));
		Vector2D c2 = Vector2D((*unknown_pole2_3).second.x, (*unknown_pole2_3).second.y) + dir4*unknown_pole_rad ;
		
		Circle unknown_pole_2(c2.x,c2.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	
	if (visibility_case2 == 7) // 1, 2, 3 & 4
	{
		x2_unknown = ((*unknown_pole2_1).second.x + (*unknown_pole2_2).second.x + (*unknown_pole2_3).second.x + (*unknown_pole2_4).second.x)/4;
		y2_unknown = ((*unknown_pole2_1).second.y + (*unknown_pole2_2).second.y + (*unknown_pole2_3).second.y + (*unknown_pole2_4).second.y)/4;
		
		Circle unknown_pole_2(x2_unknown,y2_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_2);
	}
	
	
	////////////////////////////////////////////////////////////////
	// third unknown pole
	Landmark3DMap::iterator unknown_pole3_1  = lm_map.find(33);
	Landmark3DMap::iterator unknown_pole3_2  = lm_map.find(34);
	Landmark3DMap::iterator unknown_pole3_3  = lm_map.find(35);
	Landmark3DMap::iterator unknown_pole3_4  = lm_map.find(36);
	
	int visibility_case3;	
		
	if (unknown_pole3_1 != lm_map.end() && unknown_pole3_2 != lm_map.end())
		visibility_case3 = 1; // 1 & 2
		
	if (unknown_pole3_1 != lm_map.end() && unknown_pole3_3 != lm_map.end())
		visibility_case3 = 2; // 1 & 3
		
	if (unknown_pole3_1 != lm_map.end() && unknown_pole3_4 != lm_map.end())
		visibility_case3 = 3; // 1 & 4	
		
	if (unknown_pole3_2 != lm_map.end() && unknown_pole3_3 != lm_map.end())
		visibility_case3 = 4; // 2 & 3	
	
	if (unknown_pole3_1 != lm_map.end() && unknown_pole3_3 != lm_map.end())
		visibility_case3 = 5; // 2 & 4
		
	if (unknown_pole3_1 != lm_map.end() && unknown_pole3_3 != lm_map.end())
		visibility_case3 = 6; // 3 & 4
		
	if (unknown_pole3_1 != lm_map.end() && unknown_pole3_2 != lm_map.end() && unknown_pole3_3 != lm_map.end() && unknown_pole3_4 != lm_map.end())
		visibility_case3 = 7; // all four landmarks detected
		
	
	double x3_unknown, y3_unknown;	
	if (visibility_case3 == 1) // 1 & 2
	{
		Vector2D dir2(cos((*unknown_pole3_2).second.roll),sin((*unknown_pole3_2).second.roll));
		Vector2D c3 = Vector2D((*unknown_pole3_1).second.x, (*unknown_pole3_1).second.y) + dir2*unknown_pole_rad ;
		
		Circle unknown_pole_3(c3.x,c3.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	if (visibility_case3 == 2) //1 & 3
	{
		x3_unknown = ((*unknown_pole3_1).second.x + (*unknown_pole3_3).second.x)/2;
		y3_unknown = ((*unknown_pole3_1).second.y + (*unknown_pole3_3).second.y)/2;
		
		Circle unknown_pole_3(x3_unknown,y3_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	if (visibility_case3 == 3) // 1 & 4
	{
		Vector2D dir4(cos((*unknown_pole3_4).second.roll),sin((*unknown_pole3_4).second.roll));
		Vector2D c3 = Vector2D((*unknown_pole3_1).second.x, (*unknown_pole3_1).second.y) - dir4*unknown_pole_rad ;
		
		Circle unknown_pole_3(c3.x,c3.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	if (visibility_case3 == 4) // 2 & 3
	{
		Vector2D dir3(cos((*unknown_pole3_3).second.roll),sin((*unknown_pole3_3).second.roll));
		Vector2D c3 = Vector2D((*unknown_pole3_2).second.x, (*unknown_pole3_2).second.y) + dir3*unknown_pole_rad ;
		
		Circle unknown_pole_3(c3.x,c3.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	if (visibility_case3 == 5) // 2 & 4
	{
		x3_unknown = ((*unknown_pole3_2).second.x + (*unknown_pole3_4).second.x)/2;
		y3_unknown = ((*unknown_pole3_2).second.y + (*unknown_pole3_4).second.y)/2;
		
		Circle unknown_pole_3(x3_unknown,y3_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	if (visibility_case3 == 6) // 3 & 4
	{
		Vector2D dir4(cos((*unknown_pole3_4).second.roll),sin((*unknown_pole3_4).second.roll));
		Vector2D c3 = Vector2D((*unknown_pole3_3).second.x, (*unknown_pole3_3).second.y) + dir4*unknown_pole_rad ;
		
		Circle unknown_pole_3(c3.x,c3.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	
	if (visibility_case3 == 7) // 1, 2, 3 & 4
	{
		x3_unknown = ((*unknown_pole3_1).second.x + (*unknown_pole3_2).second.x + (*unknown_pole3_3).second.x + (*unknown_pole3_4).second.x)/4;
		y3_unknown = ((*unknown_pole3_1).second.y + (*unknown_pole3_2).second.y + (*unknown_pole3_3).second.y + (*unknown_pole3_4).second.y)/4;
		
		Circle unknown_pole_3(x3_unknown,y3_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_3);
	}
	
	
	////////////////////////////////////////////////////////////////
	// fourth unknown pole
	Landmark3DMap::iterator unknown_pole4_1  = lm_map.find(37);
	Landmark3DMap::iterator unknown_pole4_2  = lm_map.find(38);
	Landmark3DMap::iterator unknown_pole4_3  = lm_map.find(39);
	Landmark3DMap::iterator unknown_pole4_4  = lm_map.find(40);
	
	int visibility_case4;	
		
	if (unknown_pole4_1 != lm_map.end() && unknown_pole4_2 != lm_map.end())
		visibility_case4 = 1; // 1 & 2
		
	if (unknown_pole4_1 != lm_map.end() && unknown_pole4_3 != lm_map.end())
		visibility_case4 = 2; // 1 & 3
		
	if (unknown_pole4_1 != lm_map.end() && unknown_pole4_4 != lm_map.end())
		visibility_case4 = 3; // 1 & 4	
		
	if (unknown_pole4_2 != lm_map.end() && unknown_pole4_3 != lm_map.end())
		visibility_case4 = 4; // 2 & 3	
	
	if (unknown_pole4_1 != lm_map.end() && unknown_pole4_3 != lm_map.end())
		visibility_case4 = 5; // 2 & 4
		
	if (unknown_pole4_1 != lm_map.end() && unknown_pole4_3 != lm_map.end())
		visibility_case4 = 6; // 3 & 4
		
	if (unknown_pole4_1 != lm_map.end() && unknown_pole4_2 != lm_map.end() && unknown_pole4_3 != lm_map.end() && unknown_pole4_4 != lm_map.end())
		visibility_case4 = 7; // all four landmarks detected
		
	
	double x4_unknown, y4_unknown;	
	if (visibility_case4 == 1) // 1 & 2
	{
		Vector2D dir2(cos((*unknown_pole4_2).second.roll),sin((*unknown_pole4_2).second.roll));
		Vector2D c4 = Vector2D((*unknown_pole4_1).second.x, (*unknown_pole4_1).second.y) + dir2*unknown_pole_rad ;
		
		Circle unknown_pole_4(c4.x,c4.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}
	
	if (visibility_case4 == 2) //1 & 3
	{
		x4_unknown = ((*unknown_pole4_1).second.x + (*unknown_pole4_3).second.x)/2;
		y4_unknown = ((*unknown_pole4_1).second.y + (*unknown_pole4_3).second.y)/2;
		
		Circle unknown_pole_4(x4_unknown,y4_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}
	
	if (visibility_case4 == 3) // 1 & 4
	{
		Vector2D dir4(cos((*unknown_pole4_4).second.roll),sin((*unknown_pole4_4).second.roll));
		Vector2D c4 = Vector2D((*unknown_pole4_1).second.x, (*unknown_pole4_1).second.y) - dir4*unknown_pole_rad ;
		
		Circle unknown_pole_4(c4.x,c4.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}
	
	if (visibility_case4 == 4) // 2 & 3
	{
		Vector2D dir3(cos((*unknown_pole4_3).second.roll),sin((*unknown_pole4_3).second.roll));
		Vector2D c4 = Vector2D((*unknown_pole4_2).second.x, (*unknown_pole4_2).second.y) + dir3*unknown_pole_rad ;
		
		Circle unknown_pole_4(c4.x,c4.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}
	
	if (visibility_case4 == 5) // 2 & 4
	{
		x4_unknown = ((*unknown_pole4_2).second.x + (*unknown_pole4_4).second.x)/2;
		y4_unknown = ((*unknown_pole4_2).second.y + (*unknown_pole4_4).second.y)/2;
		
		Circle unknown_pole_4(x4_unknown,y4_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}
	
	if (visibility_case4 == 6) // 3 & 4
	{
		Vector2D dir4(cos((*unknown_pole4_4).second.roll),sin((*unknown_pole4_4).second.roll));
		Vector2D c4 = Vector2D((*unknown_pole4_3).second.x, (*unknown_pole4_3).second.y) + dir4*unknown_pole_rad ;
		
		Circle unknown_pole_4(c4.x,c4.y,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}
	
	
	if (visibility_case4 == 7) // 1, 2, 3 & 4
	{
		x4_unknown = ((*unknown_pole4_1).second.x + (*unknown_pole4_2).second.x + (*unknown_pole4_3).second.x + (*unknown_pole4_4).second.x)/4;
		y4_unknown = ((*unknown_pole4_1).second.y + (*unknown_pole4_2).second.y + (*unknown_pole4_3).second.y + (*unknown_pole4_4).second.y)/4;
		
		Circle unknown_pole_4(x4_unknown,y4_unknown,unknown_pole_rad);
		obstacles.push_back(unknown_pole_4);
	}


	return obstacles;

}



vector<Circle> ObstacleProcessor::getObstacles2(vector<Landmark3D> v)
{
	double dis_thres = unknown_pole_rad;
	
	
	for (int i=0; i<v.size(); i++)
	{
		Landmark3D lm = v[i];
		
		if (lm.is_known) continue;
		
		double ang = lm.roll + M_PI/2;
		Vector2D dir(cos(ang),sin(ang));
		
		Vector2D c = Vector2D(lm.x,lm.y) + dir*unknown_pole_rad;
		
		bool already_added = false;
		for(int j=0; j<obstacles2.size();j++)
		{
			Circle* obs = &obstacles2[j];	
			Vector2D obs_p;
			if (obs->type == CIRCLE)
			{
				Circle* obs_c = dynamic_cast<Circle*> (obs);
				obs_p.x = obs_c->center.x;
				obs_p.y = obs_c->center.y;
				
			}
			if(squaredDistance(obs_p,c) < dis_thres)
			{
				double x = (obs_p.x*obs->weight + c.x)/(obs->weight+1);
				double y = (obs_p.y*obs->weight + c.y)/(obs->weight+1);
				Circle corrected_circle(x,y,unknown_pole_rad);
				corrected_circle.weight = obs->weight+1;
				obstacles2[j] = corrected_circle;
			}

		
		}
		
		if (!already_added)
		{
			Circle new_circle(c.x,c.y,unknown_pole_rad);
			new_circle.weight = 1;
			obstacles2.push_back(new_circle);
		
		}
		
	
	
	}

	return obstacles2;
}*/

vector<Obstacle2D*> ObstacleProcessor::getObstacles(vector<Landmark3D> v)
{

    cout << "obstacle processor " << endl;

    int id_base_wall_rectangles = 3000;
	
    //int id_base_poles_circles = 4000;
	
    //int num_poles_added = 0;

    for (unsigned int i=0;i<detectedObstacles.size();i++)
        delete detectedObstacles[i];

    detectedObstacles.clear();

	Landmark3DMap lm_map;
	for (int i=0; i<v.size(); i++)
	{
		Landmark3D lm = v[i];
		int key = lm.id;
		lm_map[key]=lm;
	}
	
	//////////////////////////////////////////////////////////////////////////
	
	//Option 1
	
	if (windows_distribution_option == 1)
	{
		//small window
		Landmark3DMap::iterator large_window_1  = lm_map.find(1);
		Landmark3DMap::iterator large_window_2  = lm_map.find(2);
		Landmark3DMap::iterator large_window_3  = lm_map.find(3);
		Landmark3DMap::iterator large_window_4  = lm_map.find(4);
	
		//large window
		Landmark3DMap::iterator small_window_1  = lm_map.find(5);
		Landmark3DMap::iterator small_window_2  = lm_map.find(6);
		Landmark3DMap::iterator small_window_3  = lm_map.find(7);
		Landmark3DMap::iterator small_window_4  = lm_map.find(8);
	
		if (large_window_1 != lm_map.end() && large_window_2 != lm_map.end())
		{
			double length1 = ((*large_window_1).second.x + (*large_window_2).second.x)/2;
			double width1 = wall_width;
			Pose p(length1/2,wall_width/2,0);
            Rectangle* r1 = new Rectangle(p,length1,width1);
            r1->id = id_base_wall_rectangles;
            detectedObstacles.push_back(r1);
		
			if (large_window_3 != lm_map.end() && large_window_4 != lm_map.end() && small_window_1 != lm_map.end() && small_window_2 != lm_map.end())
			{
				double length2 = ((*small_window_1).second.x + (*small_window_2).second.x)/2 - ((*large_window_4).second.x + (*large_window_3).second.x)/2;
				double width2 = wall_width;
				Pose p2(length1 + large_window_length + length2/2,wall_width/2,0);
                Rectangle* r2 = new Rectangle(p2,length2,width2);
                r2->id = id_base_wall_rectangles+1;
                detectedObstacles.push_back(r2);
			
				//if (small_window_3 != lm_map.end() && small_window_4 != lm_map.end())
				{
					double length3 = wall_length - (length1 + large_window_length + length2 + small_window_length);
					double width3 = wall_width;
					Pose p3(length1 + large_window_length + length2 + small_window_length + length3/2,wall_width/2,0);
                    Rectangle* r3 = new Rectangle(p3,length3,width3);
                    r3->id = id_base_wall_rectangles+2;
                    detectedObstacles.push_back(r3);
	
				}
			
			
	
			}
	
		}
	
	}

	//////////////////////////////////////////////////////////////////////////
	
	//Option 2
	
    /*if (windows_distribution_option == 2)
	{
		//small window
		Landmark3DMap::iterator large_window_1  = lm_map.find(1);
		Landmark3DMap::iterator large_window_2  = lm_map.find(2);
		Landmark3DMap::iterator large_window_3  = lm_map.find(3);
		Landmark3DMap::iterator large_window_4  = lm_map.find(4);
	
		//large window
		Landmark3DMap::iterator small_window_1  = lm_map.find(5);
		Landmark3DMap::iterator small_window_2  = lm_map.find(6);
		Landmark3DMap::iterator small_window_3  = lm_map.find(7);
		Landmark3DMap::iterator small_window_4  = lm_map.find(8);
	
		if (!(*large_window_1).second.is_known && !(*large_window_2).second.is_known && !(*large_window_3).second.is_known && !(*large_window_4).second.is_known && !(*small_window_1).second.is_known && !(*small_window_2).second.is_known && !(*small_window_3).second.is_known && !(*small_window_4).second.is_known)
		{
	
			if (large_window_1 != lm_map.end())
			{
				double length1 = (*large_window_1).second.x;
				double width1 = wall_width;
				Pose p(length1/2,wall_width/2,0);
                Rectangle* r1 = new Rectangle(p,length1,width1);
                r1->id = id_base_wall_rectangles;
                detectedObstacles.push_back(r1);
		
				if (large_window_4 != lm_map.end() && small_window_1 != lm_map.end())
				{
					double length2 = (*small_window_1).second.x - (*large_window_4).second.x;
					double width2 = wall_width;
					Pose p2(length1 + large_window_length + length2/2,wall_width/2,0);
                    Rectangle* r2 = new Rectangle(p2,length2,width2);
                    r2->id = id_base_wall_rectangles+1;
                    detectedObstacles.push_back(r2);
			
					//if (small_window_4 != lm_map.end())
					{
						double length3 = wall_length - (length1 + large_window_length + length2 + small_window_length);
						double width3 = wall_width;
						Pose p3(length1 + large_window_length + length2 + small_window_length + length3/2,wall_width/2,0);
                        Rectangle* r3 = new Rectangle(p3,length3,width3);
                        r3->id = id_base_wall_rectangles+2;
                        detectedObstacles.push_back(r3);
	
					}
			
			
	
				}
	
			}
		}
	
    }*/

    cout << "windows option " << windows_distribution_option << endl;

    if (windows_distribution_option == 2)
    {
        cout << "here " << endl;
        //large window
        Landmark3DMap::iterator large_window_1  = lm_map.find(1);
        Landmark3DMap::iterator large_window_2  = lm_map.find(2);
        Landmark3DMap::iterator large_window_3  = lm_map.find(3);
        Landmark3DMap::iterator large_window_4  = lm_map.find(4);

        //small window
        Landmark3DMap::iterator small_window_1  = lm_map.find(5);
        Landmark3DMap::iterator small_window_2  = lm_map.find(6);
        Landmark3DMap::iterator small_window_3  = lm_map.find(7);
        Landmark3DMap::iterator small_window_4  = lm_map.find(8);

        bool large_window_detected = false;
        bool small_window_detected = false;


        if((large_window_1!= lm_map.end() || large_window_2!= lm_map.end()) && (large_window_3!= lm_map.end() || large_window_4!= lm_map.end()))
            large_window_detected = true;

        if((small_window_1!= lm_map.end() || small_window_2!= lm_map.end()) && (small_window_3!= lm_map.end() || small_window_4!= lm_map.end()))
            small_window_detected = true;

        if (large_window_detected) cout << "large detected " << endl;
        if (small_window_detected) cout << "small detected " << endl;

        //one rectangle 0
        if (!large_window_detected && !small_window_detected) //windows not detected, wall obstacle
        {
            Pose p_wall(wall_length/2,0,0);
            Rectangle* wall = new Rectangle(p_wall,wall_length,wall_width);
            wall->id = id_base_wall_rectangles; //whole wall
            detectedObstacles.push_back(wall);

        }

        // two rectangles 1,2
        if(large_window_detected && !small_window_detected)
        {
            double length1;
            if (large_window_1!= lm_map.end() && large_window_2!= lm_map.end() )
                length1 = ((*large_window_1).second.x + (*large_window_2).second.x)/2 + aruco_width;
            else
            {
                if(large_window_1!= lm_map.end())
                    length1 = (*large_window_1).second.x+ aruco_width;
                if(large_window_2!= lm_map.end())
                    length1 = (*large_window_2).second.x+ aruco_width;
            }

            double width1 = wall_width;
            Pose p(length1/2,wall_width/2,0);
            Rectangle* r1 = new Rectangle(p,length1,width1);
            r1->id = id_base_wall_rectangles+1;
            detectedObstacles.push_back(r1);

            double dis;
            if (large_window_3!= lm_map.end() && large_window_4!= lm_map.end() )
                dis = ((*large_window_3).second.x + (*large_window_4).second.x)/2 - aruco_width;
            else
            {
                if(large_window_3!= lm_map.end())
                    dis = (*large_window_3).second.x- aruco_width;
                if(large_window_4!= lm_map.end())
                    dis = (*large_window_4).second.x- aruco_width;
            }

            double length2 = wall_length-dis;
            double width2 = wall_width;
            Pose p2(dis+length2/2,wall_width/2,0);
            Rectangle* r2 = new Rectangle(p2,length2,width2);
            r2->id = id_base_wall_rectangles+2;
            detectedObstacles.push_back(r2);

        }

        // two rectangles 3,4
        if(!large_window_detected && small_window_detected)
        {
            double small_window_left;
            if (small_window_1!= lm_map.end() && small_window_2!= lm_map.end() )
                small_window_left = ((*small_window_1).second.x + (*small_window_2).second.x)/2 + aruco_width;
            else
            {
                if(small_window_1!= lm_map.end())
                    small_window_left = (*small_window_1).second.x+ aruco_width;
                if(small_window_2!= lm_map.end())
                    small_window_left = (*small_window_2).second.x+ aruco_width;
            }

            double length1 = small_window_left;
            double width1 = wall_width;
            Pose p(length1/2,wall_width/2,0);
            Rectangle* r1 = new Rectangle(p,length1,width1);
            r1->id = id_base_wall_rectangles+3;
            detectedObstacles.push_back(r1);

            double small_window_right;
            if (small_window_3!= lm_map.end() && small_window_4!= lm_map.end() )
                small_window_right = ((*small_window_3).second.x + (*small_window_4).second.x)/2 + aruco_width;
            else
            {
                if(small_window_3!= lm_map.end())
                    small_window_right = (*small_window_3).second.x- aruco_width;
                if(small_window_4!= lm_map.end())
                    small_window_right = (*small_window_4).second.x- aruco_width;
            }

            double length2 = wall_length-small_window_right;
            double width2 = wall_width;
            Pose p2(small_window_right+length2/2,wall_width/2,0);
            Rectangle* r2 = new Rectangle(p2,length2,width2);
            r2->id = id_base_wall_rectangles+4;
            detectedObstacles.push_back(r2);

        }


        // three rectangles 5,6,7
        if(large_window_detected && small_window_detected)
        {
            double length1;
            if (large_window_1!= lm_map.end() && large_window_2!= lm_map.end() )
                length1 = ((*large_window_1).second.x + (*large_window_2).second.x)/2 + aruco_width;
            else
            {
                if(large_window_1!= lm_map.end())
                    length1 = (*large_window_1).second.x+ aruco_width;
                if(large_window_2!= lm_map.end())
                    length1 = (*large_window_2).second.x+ aruco_width;
            }

            double width1 = wall_width;
            Pose p(length1/2,wall_width/2,0);
            Rectangle* r1 = new Rectangle(p,length1,width1);
            r1->id = id_base_wall_rectangles+5;
            detectedObstacles.push_back(r1);

            double large_window_right_dis;
            if (large_window_3!= lm_map.end() && large_window_4!= lm_map.end() )
                large_window_right_dis = ((*large_window_3).second.x + (*large_window_4).second.x)/2 - aruco_width;
            else
            {
                if(large_window_3!= lm_map.end())
                    large_window_right_dis = (*large_window_3).second.x- aruco_width;
                if(large_window_4!= lm_map.end())
                    large_window_right_dis = (*large_window_4).second.x- aruco_width;
            }

            double small_window_left_dis;
            if (small_window_1!= lm_map.end() && small_window_2!= lm_map.end() )
                small_window_left_dis = ((*small_window_1).second.x + (*small_window_2).second.x)/2 - aruco_width;
            else
            {
                if(small_window_1!= lm_map.end())
                    small_window_left_dis = (*small_window_1).second.x+ aruco_width;
                if(small_window_2!= lm_map.end())
                    small_window_left_dis = (*small_window_2).second.x+ aruco_width;
            }

            double length2 = small_window_left_dis - large_window_right_dis;
            double width2 = wall_width;
            Pose p2(large_window_right_dis+length2/2,wall_width/2,0);
            Rectangle* r2 = new Rectangle(p2,length2,width2);
            r2->id = id_base_wall_rectangles+6;
            detectedObstacles.push_back(r2);

            double small_window_right;
            if (small_window_3!= lm_map.end() && small_window_4!= lm_map.end() )
                small_window_right = ((*small_window_3).second.x + (*small_window_4).second.x)/2 - aruco_width;
            else
            {
                if(small_window_3!= lm_map.end())
                    small_window_right = (*small_window_3).second.x- aruco_width;
                if(small_window_4!= lm_map.end())
                    small_window_right = (*small_window_4).second.x- aruco_width;
            }

            double length3 = wall_length - small_window_right;
            double width3 = wall_width;
            Pose p3(small_window_right + length3/2,wall_width/2,0);
            Rectangle* r3 = new Rectangle(p3,length3,width3);
            r3->id = id_base_wall_rectangles+7;
            detectedObstacles.push_back(r3);

        }


    }

    //////////////////////////////////////////////////////////////////////////

    //double dis_thres = unknown_pole_rad;
	
	for (int i=0; i<v.size(); i++)
	{
		Landmark3D lm = v[i];

        cout<<"searching id="<<lm.id<<endl;
		
        //if (lm.id <= num_landmarks_wind) continue; // add known poles too
		
        bool flagObstacleDefined=false;
        int obstaclePositionInVector;
        for(int k=0;k<existingPoles.size();k++)
        {
            set<int>::iterator it = existingPoles[k].arucoIds.find(lm.id);
            if (it != existingPoles[k].arucoIds.end())
            {
                flagObstacleDefined=true;
                obstaclePositionInVector=k;
                //cout<<"Obstacle found. id = "<<existingPoles[k].id<<"; aruco id = "<<lm.id<<endl;
                break;
            }

        }

        if(!flagObstacleDefined)
            continue;



        double ang = lm.roll + M_PI;
        Vector2D dir(cos(ang),sin(ang));
		
		  Vector2D c;
        c = Vector2D(lm.x,lm.y) + dir*existingPoles[obstaclePositionInVector].radius;

		
        bool already_added = false;
        for(int j=0; j<detectedObstacles.size();j++)
		  {
            Obstacle2D* obs = detectedObstacles[j];
            if (obs->id == existingPoles[obstaclePositionInVector].id)
            {
		         cout<<" obstacle already added!"<<endl;
		         already_added = true;
		         
					Vector2D obs_p;
					int obs_id;
					if (obs->type == CIRCLE)
					{
		             Circle* obs_c = dynamic_cast<Circle*> (obs);

						obs_id = obs_c->id;
						obs_p.x = obs_c->center.x;
						obs_p.y = obs_c->center.y;

					}

		         cout << "existing obstacle " << obs_p.x << " " << obs_p.y << endl;
		         cout << "new hypothesis " << c.x << " " << c.y << endl;

					double x = (obs_p.x*obs->weight + c.x)/(obs->weight+1);
					double y = (obs_p.y*obs->weight + c.y)/(obs->weight+1);
			      double radius = existingPoles[obstaclePositionInVector].radius;
			      
					Circle corrected_circle(x,y,radius);
					corrected_circle.weight = obs->weight+1;
					corrected_circle.id = obs_id;
					corrected_circle.arucoIds.insert(lm.id);
			      (*(detectedObstacles[j])) = corrected_circle;

				
				}


		}
		
		if (!already_added)
		{
            double radius = existingPoles[obstaclePositionInVector].radius;
            Circle* new_circle = new Circle(c.x,c.y,radius);
            new_circle->weight = 1;
            new_circle->id = existingPoles[obstaclePositionInVector].id;
            new_circle->arucoIds.insert(lm.id);
            detectedObstacles.push_back(new_circle);
		
		}
		
	
	
	}

    return detectedObstacles;
}

