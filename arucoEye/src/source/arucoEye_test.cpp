//////////////////////////////////////////////////////
//  discreteSearch.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 28, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>

//Vector
//std::vector
#include <vector>


//Opencv
#include <opencv2/opencv.hpp>


//Aruco
#include "aruco.h"
//ROS aruco
#include "arucoEye.h"




#define DEBUG_ARUCO_EYE



//configurations
const bool ARUCO_EYE_CONFIG_enableErosion=false;
const aruco::MarkerDetector::ThresholdMethods ARUCO_EYE_CONFIG_thresholdMethod=aruco::MarkerDetector::ADPT_THRES;
const double ARUCO_EYE_CONFIG_ThresParam1=7;
const double ARUCO_EYE_CONFIG_ThresParam2=7;
const aruco::MarkerDetector::CornerRefinementMethod ARUCO_EYE_CONFIG_methodCornerRefinement=aruco::MarkerDetector::LINES;
const int ARUCO_EYE_CONFIG_ThePyrDownLevel=0;
const float ARUCO_EYE_CONFIG_minSize=0.03;//0.03;
const float ARUCO_EYE_CONFIG_maxSize=0.5;//0.5;



using namespace std;



int main(int argc,char **argv)
{
    cout<<"[TEST] Init test droneArucoRetina"<<endl;

    //ArucoRetina
    ArucoEye ArucoRetina;


    //Camera
    cv::VideoCapture TheVideoCapturer;
    cv::Mat TheInputImage;


    //Image display
	int waitTime=1;
    const std::string windowName="Test Window";


	try
	{

		//read from camera or from  file
        TheVideoCapturer.open(0);

		//check video is open
		if (!TheVideoCapturer.isOpened())
		{
            cerr<<"[TEST] Could not open video"<<endl;
		    return -1;

		}


		//read first image to get the dimensions
        //TheVideoCapturer>>ArucoRetina.InputImage;

		//Open output video
		//if ( TheOutVideoFilePath!="")  // define the videos you want to output into the specified files
        //    VWriter.open(TheOutVideoFilePath,CV_FOURCC('M','J','P','G'),15,InputImage.size());


        //Start retina
        //ArucoRetina.start();
        const std::string arucoList="/home/joselu/workspace/ros/quadrotor/stack/arucoEye/config/arUcoList.xml";
        const std::string cameraCalibration="/home/joselu/workspace/ros/quadrotor/stack/arucoEye/config/camera.yml";

        if(!ArucoRetina.configureArucoDetector(ARUCO_EYE_CONFIG_enableErosion,
                                               ARUCO_EYE_CONFIG_thresholdMethod,
                                               ARUCO_EYE_CONFIG_ThresParam1,
                                               ARUCO_EYE_CONFIG_ThresParam2,
                                               ARUCO_EYE_CONFIG_methodCornerRefinement,
                                               ARUCO_EYE_CONFIG_ThePyrDownLevel,
                                               ARUCO_EYE_CONFIG_minSize,
                                               ARUCO_EYE_CONFIG_maxSize))
        {
            cout<<"Error configuring aruco detector"<<endl;
            return 0;
        }

        ArucoRetina.setArucoList(arucoList);
        ArucoRetina.setCameraParameters(cameraCalibration);

        unsigned int numCodesDetected, numCodesReconstructed;


		//Create gui
        cv::namedWindow(windowName,1);
		

        //Key
		char key=0;


		//capture until press ESC or until the end of the video
        while ( key!=27 && TheVideoCapturer.grab())
		{
			
			//Capture image
            TheVideoCapturer.retrieve( TheInputImage);
            if(!ArucoRetina.setInputImage(TheInputImage))
                continue;


            //Run retina
            if(!ArucoRetina.run(numCodesDetected,numCodesReconstructed))
                continue;

#ifdef DEBUG_ARUCO_EYE
            cout<<"[TEST] ----------------"<<endl;
            cout<<"[TEST] codesDetected="<<numCodesDetected<<"; codesReconstructed="<<numCodesReconstructed<<endl;
#endif

            //Get 3D information
            std::vector<aruco::Marker> TheMarkers;

            if(!ArucoRetina.getMarkers3DInformation(TheMarkers))
            {
                cout<<"[TEST] !!Error getting 3D information"<<endl;
                continue;
            }


            for(unsigned int i=0; i<TheMarkers.size();i++)
            {
                cv::Mat HomogMat;

                homogeneusTransformation::createMatHomogFromVecs(HomogMat,TheMarkers[i].Tvec,TheMarkers[i].Rvec);

                cv::Mat RotVec2;
                cv::Mat TraVec2;

                homogeneusTransformation::calculateVecsFromMatHomog(TraVec2,RotVec2,HomogMat);

                cv::Mat YPRVec;
                cv::Mat TraVec3;
                homogeneusTransformation::calculateTraVecYPRFromMatHomog(&YPRVec,&TraVec3,HomogMat);

                cv::Mat HomogMatRec;
                homogeneusTransformation::calculateMatHomogFromYPRVecTraVec(&HomogMatRec,YPRVec,TraVec2);


#ifdef DEBUG_ARUCO_EYE
                  //Sacamos valores
                std::cout<<"[TEST] ->Marker id="<<TheMarkers[i].id<<endl;
                std::cout<<"[TEST]   Trans Vec="<<TheMarkers[i].Tvec<<endl;
                std::cout<<"[TEST]   Rot Vec="<<TheMarkers[i].Rvec<<endl;

                //cout<<" *Trans Vec="<<TraVec2<<endl;
                //cout<<" *Rot Vec="<<RotVec2<<endl;
                std::cout<<"[TEST]   Homog Mat="<<HomogMat<<endl;
                //std::cout<<" *------------------------- YPR=" << YPRVec.mul(180.0/3.14) << endl;
                //std::cout<<" *------------------------- Trans Vec=" << TraVec3 << endl;
                //std::cout<<" *Rec Homog Mat="<<HomogMatRec<<endl;

#endif

            }


            //Draw
            key=ArucoRetina.drawDetectedArucoCodes(windowName,waitTime,true,true);

			
		}
        return 1;


    }
    catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

}


