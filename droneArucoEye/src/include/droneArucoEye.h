//////////////////////////////////////////////////////
//  droneArucoEye.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef _DRONE_ARUCO_EYE_H
#define _DRONE_ARUCO_EYE_H



//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>

//Vector
//std::vector
#include <vector>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>



//Aruco
#include "aruco.h"
//Aruco JL Lib
#include "arucoEye.h"

//reference frames
#include "referenceFrames.h"


//#include "xmlfilereader.h"




#define VERBOSE_DRONE_ARUCO_EYE



//configurations
const bool ARUCO_EYE_CONFIG_enableErosion=false;
const aruco::MarkerDetector::ThresholdMethods ARUCO_EYE_CONFIG_thresholdMethod=aruco::MarkerDetector::ADPT_THRES;
const double ARUCO_EYE_CONFIG_ThresParam1=7;
const double ARUCO_EYE_CONFIG_ThresParam2=7;
const aruco::MarkerDetector::CornerRefinementMethod ARUCO_EYE_CONFIG_methodCornerRefinement=aruco::MarkerDetector::LINES;
const int ARUCO_EYE_CONFIG_ThePyrDownLevel=0;
const float ARUCO_EYE_CONFIG_minSize=0.045;//0.03;
const float ARUCO_EYE_CONFIG_maxSize=0.5;//0.5;





/////////////////////////////////////////
// Class DroneMarker
//
//   Description
//
/////////////////////////////////////////
class DroneMarker
{
protected:
    int id;
    cv::Mat HomogMat;

public:
    DroneMarker();
    ~DroneMarker();

public:
    int setDroneMarker(int idIn, const cv::Mat &HomogMatIn);
    int getDroneMarker(int& idOut, cv::Mat& HomogMatOut);

};




/////////////////////////////////////////
// Class DroneArucoEye
//
//   Description
//
/////////////////////////////////////////
class DroneArucoEye
{

    //MyArucoEye
protected:
    ArucoEye MyArucoEye;


private:
    // reference frame related matrixes, see documentation/IMAV_Frames/IMAV13_Frames_documentation.pdf
    cv::Mat matHomog_aruco_GMR_wrt_drone_GMR;       // as required by ArUCo SLAM module
    cv::Mat matHomog_drone_ALF_wrt_drone_GMR;       // fixed reference change, Reference frame change to cog of the drone
    cv::Mat matHomog_aruco_ALF_wrt_drone_ALF;       // as returned by the ArUCo library
    cv::Mat matHomog_aruco_GMR_wrt_aruco_ALF;       // fixed reference change

    //Images input
public:
    int setInputImage(cv::Mat InputImage);

    //Reconstruction
protected:
    unsigned int numCodesDetected;
    unsigned int numCodesReconstructed;
    std::vector<DroneMarker> TheDroneMarkers;

public:
    int getDroneMarkerI(unsigned int markerI, int &idMarker, cv::Mat& matHomog_aruco_GMR_wrt_drone_GMR_out);

    //Drawing
public:
    int drawArucoCodes(std::string windowName, int waitingTime=1, bool drawDetectedCodes=true, bool draw3DReconstructedCodes=true);


    //Constructors
public:
    DroneArucoEye();
    ~DroneArucoEye();

public:
    int init();
    int close();

    //Configure
 public:
    int configureArucoEye(std::string arucoListFile, std::string cameraCalibrationFile);

    //Position of the camera in the drone
public:
    int setCameraInTheDrone(double x, double y, double z, double yaw, double pitch, double roll);
    int setCameraInTheDrone(std::string droneConfigurationFile,int idCamera=0);

    //Reset
public:
    bool reset();
    //Start
public:
    bool start();
    //Stop
public:
    bool stop();

    //Run
public:
    bool run(unsigned int &numCodesReconstructedOut);
};




#endif
