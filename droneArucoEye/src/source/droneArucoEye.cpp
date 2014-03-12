//////////////////////////////////////////////////////
//  droneArucoEye.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneArucoEye.h"



using namespace std;


///////////// DroneMarker ///////////////////
DroneMarker::DroneMarker()
{

    return;
}

DroneMarker::~DroneMarker()
{
    HomogMat.release();

    return;
}


int DroneMarker::setDroneMarker(int idIn, const cv::Mat& HomogMatIn)
{
    id=idIn;
    HomogMatIn.copyTo(HomogMat);

    return 1;
}

int DroneMarker::getDroneMarker(int& idOut, cv::Mat& HomogMatOut)
{
    idOut=id;
    HomogMat.copyTo(HomogMatOut);

    return 1;
}










////////// Drone Aruco Eye ///////////
DroneArucoEye::DroneArucoEye()
{
    init();
    return;
}


DroneArucoEye::~DroneArucoEye()
{
    close();
    return;
}

int DroneArucoEye::init()
{
    if(!MyArucoEye.init())
        return 0;

    //Matrix for change of reference
    matHomog_aruco_GMR_wrt_drone_GMR = cv::Mat::eye(4,4,CV_32F);
    matHomog_drone_ALF_wrt_drone_GMR = cv::Mat::eye(4,4,CV_32F);
    matHomog_aruco_ALF_wrt_drone_ALF = cv::Mat::eye(4,4,CV_32F);
    matHomog_aruco_GMR_wrt_aruco_ALF = cv::Mat::eye(4,4,CV_32F);

    // assign value to matHomog_aruco_GMR_wrt_aruco_ALF
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw    = (M_PI/180.0)*(+90.0); // rad
    double pitch  = 0.0;
    double roll   = (M_PI/180.0)*(-90.0); // rad
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_aruco_GMR_wrt_aruco_ALF, x, y, z, yaw, pitch, roll);
    return 1;
}


int DroneArucoEye::close()
{
    if(!MyArucoEye.close())
        return 0;

    return 1;
}


int DroneArucoEye::setCameraInTheDrone(double x, double y, double z, double yaw, double pitch, double roll)
{
    //Set camera parameters
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_ALF_wrt_drone_GMR, x, y, z, yaw, pitch, roll);

    return 1;
}


int DroneArucoEye::setCameraInTheDrone(std::string droneConfigurationFile, int idCamera)
{

    double x, y, z, yaw, pitch, roll;
    bool cameraFound=false;

    //Jesus method. Temporary removed, because it cannot handle multiple cameras. TODO!
//    try
//    {
//        XMLFileReader my_xml_reader(droneConfigurationFile);

//        int idCamera_fromfile = my_xml_reader.readIntValue( {"droneConfiguration","camera","id"} );
//        if(idCamera_fromfile == idCamera)
//            cameraFound=true;

//        x = my_xml_reader.readDoubleValue( {"droneConfiguration","camera","position","x"} );
//        y = my_xml_reader.readDoubleValue( {"droneConfiguration","camera","position","y"} );
//        z = my_xml_reader.readDoubleValue( {"droneConfiguration","camera","position","z"} );
//        yaw   = (M_PI/180.0)*my_xml_reader.readDoubleValue( {"droneConfiguration","camera","position","yaw"} );
//        pitch = (M_PI/180.0)*my_xml_reader.readDoubleValue( {"droneConfiguration","camera","position","pitch"} );
//        roll  = (M_PI/180.0)*my_xml_reader.readDoubleValue( {"droneConfiguration","camera","position","roll"} );
//    }
//    catch ( cvg_XMLFileReader_exception &e)
//    {
//        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
//    }

    //JL Method. Unsafe.
    //read xml
    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(droneConfigurationFile);
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] I cannot open xml file: "<<droneConfigurationFile<<endl;
#endif
        return 0;
    }

    ///droneConfiguration
    pugi::xml_node droneConfiguration = doc.child("droneConfiguration");

    //vars
    std::string readingValue;

    //search for the camera
    for(pugi::xml_node camera = droneConfiguration.child("camera");camera; camera = camera.next_sibling("camera"))
    {
        int id;
        readingValue=camera.child_value("id");
        istringstream convertid(readingValue);
        convertid>>id;

        if(id!=idCamera)
            continue;
        else
           {
            cameraFound=true;

            //read camera parameters
            readingValue=camera.child("position").child_value("x");
            istringstream convertx(readingValue);
            convertx>>x;

            readingValue=camera.child("position").child_value("y");
            istringstream converty(readingValue);
            converty>>y;

            readingValue=camera.child("position").child_value("z");
            istringstream convertz(readingValue);
            convertz>>z;

            readingValue=camera.child("position").child_value("yaw");
            istringstream convertyaw(readingValue);
            convertyaw>>yaw;
            yaw   *= (M_PI/180.0);

            readingValue=camera.child("position").child_value("pitch");
            istringstream convertpitch(readingValue);
            convertpitch>>pitch;
            pitch *= (M_PI/180.0);

            readingValue=camera.child("position").child_value("roll");
            istringstream convertroll(readingValue);
            convertroll>>roll;
            roll  *= (M_PI/180.0);

            break;
        }

    }

    if(!cameraFound)
        return 0;

    std::cout << "DroneArucoEye::setCameraInTheDrone" << std::endl;
    std::cout << "x:" << x << std::endl;
    std::cout << "y:" << y << std::endl;
    std::cout << "z:" << z << std::endl;
    std::cout << "yaw:" <<   yaw*(180.0/M_PI) << std::endl;
    std::cout << "pitch:" << pitch*(180.0/M_PI) << std::endl;
    std::cout << "roll:" <<  roll*(180.0/M_PI) << std::endl;

    //Set camera parameters
    if(!setCameraInTheDrone(x,y,z,yaw,pitch,roll))
        return 0;

    return 1;
}


int DroneArucoEye::configureArucoEye(std::string arucoListFile, std::string cameraCalibrationFile)
{

    //configure aruco detector
    if(!MyArucoEye.configureArucoDetector(ARUCO_EYE_CONFIG_enableErosion,
                                          ARUCO_EYE_CONFIG_thresholdMethod,
                                          ARUCO_EYE_CONFIG_ThresParam1,
                                          ARUCO_EYE_CONFIG_ThresParam2,
                                          ARUCO_EYE_CONFIG_methodCornerRefinement,
                                          ARUCO_EYE_CONFIG_ThePyrDownLevel,
                                          ARUCO_EYE_CONFIG_minSize,
                                          ARUCO_EYE_CONFIG_maxSize) )
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] Error configuring Aruco Retina"<<endl;
#endif
        return 0;
    }

    //set aruco list
    if(!MyArucoEye.setArucoList(arucoListFile))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] Error configuring Aruco Retina"<<endl;
#endif
        return 0;
    }

    //set camera parameters
    if(!MyArucoEye.setCameraParameters(cameraCalibrationFile))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] Error configuring Aruco Retina"<<endl;
#endif
        return 0;
    }

    //End
    return 1;
}


//Reset
bool DroneArucoEye::reset()
{

    return true;
}

//Start
bool DroneArucoEye::start()
{

    return true;
}

//Stop
bool DroneArucoEye::stop()
{

    return true;
}


int DroneArucoEye::setInputImage(cv::Mat InputImage)
{
    if(!MyArucoEye.setInputImage(InputImage))
        return 0;
    return 1;
}


int DroneArucoEye::getDroneMarkerI(unsigned int markerI, int &idMarker, cv::Mat& matHomog_aruco_GMR_wrt_drone_GMR_out)
{
    if(markerI>=TheDroneMarkers.size())
        return 0;

    if(!TheDroneMarkers[markerI].getDroneMarker(idMarker,matHomog_aruco_GMR_wrt_drone_GMR_out))
        return 0;

    return 1;
}


//Run
bool DroneArucoEye::run(unsigned int& numCodesReconstructedOut)
{
    //Init
    TheDroneMarkers.clear();

    //Run ArucoEye
    numCodesReconstructedOut=0;
    if(!MyArucoEye.run(numCodesDetected,numCodesReconstructed))
        return false;
    else
    {
        numCodesReconstructedOut=numCodesReconstructed;
    }

#ifdef VERBOSE_DRONE_ARUCO_EYE
    cout<<"[DAE] numCodesReconstructed="<<numCodesReconstructed<<endl;
#endif

    //Receive 3D poses
    int idMarker;
    cv::Mat RvecMarker, TvecMarker;
    for(unsigned int i=0;i<numCodesReconstructed;i++)
    {
        DroneMarker OneDroneMarker;
        //Read Marker
        if(!MyArucoEye.getMarkersI3DInformation(i,idMarker,RvecMarker,TvecMarker))
            return 0;

        //Transform to center of gravity of the drone
        homogeneusTransformation::createMatHomogFromVecs( matHomog_aruco_ALF_wrt_drone_ALF, TvecMarker, RvecMarker);
        matHomog_aruco_GMR_wrt_drone_GMR = matHomog_drone_ALF_wrt_drone_GMR*
                                           matHomog_aruco_ALF_wrt_drone_ALF*
                                           matHomog_aruco_GMR_wrt_aruco_ALF;

#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] marker id="<<idMarker<<"; Rvec="<<RvecMarker<<"; Tvec="<<TvecMarker<<endl;
        cout<<"[DAE] marker id="<<idMarker<<"; matHomog_aruco_GMR_wrt_drone_GMR="<<matHomog_aruco_GMR_wrt_drone_GMR<<endl;
#endif

        OneDroneMarker.setDroneMarker(idMarker,matHomog_aruco_GMR_wrt_drone_GMR);

//        std::cout << "DroneArucoEye::run, i:" << i << std::endl;
//        {
//            int idOut; cv::Mat HomogMatOut;
//            OneDroneMarker.getDroneMarker( idOut, HomogMatOut);
//            std::cout << "idOut" << idOut << std::endl;
//            std::cout << "HomogMatOut" << HomogMatOut << std::endl;
//        }

        //Store
        TheDroneMarkers.push_back(OneDroneMarker);

//        std::cout << "DroneArucoEye::run (after push_back()), i:" << i << std::endl;
//        for(auto it:TheDroneMarkers)
//        {
//            int idOut; cv::Mat HomogMatOut;
//            it.getDroneMarker( idOut, HomogMatOut);
//            std::cout << "idOut" << idOut << std::endl;
//            std::cout << "HomogMatOut" << HomogMatOut << std::endl;
//        }

    }
    //end
    return true;
}


int DroneArucoEye::drawArucoCodes(std::string windowName, int waitingTime, bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    return MyArucoEye.drawDetectedArucoCodes(windowName,waitingTime,drawDetectedCodes,draw3DReconstructedCodes);
}
