//////////////////////////////////////////////////////
//  arucoEye.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 28, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "arucoEye.h"



using namespace std;




/////////////////// ArucoCodeDefinition ////////////////
ArucoCodeDefinition::ArucoCodeDefinition()
{
    id=-1;
    size=-1;
    //type=-1;

    return;
}

ArucoCodeDefinition::~ArucoCodeDefinition()
{

    return;
}


int ArucoCodeDefinition::setArucoCode(int idIn, double sizeIn)
{
    id=idIn;
    size=sizeIn;
    return 1;
}

int ArucoCodeDefinition::getId()
{
    return id;
}

double ArucoCodeDefinition::getSize()
{
    return size;
}




/////////////// ArucoListDefinition ///////////////////

ArucoListDefinition::ArucoListDefinition()
{
    return;
}


ArucoListDefinition::~ArucoListDefinition()
{
    ArucoListDefinitionCodes.clear();

    return;
}

int ArucoListDefinition::loadListFromXmlFile(std::string filePath)
{
    int noError=1;

    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(filePath);
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        cout<<"I cannot open xml file: "<<filePath<<endl;
        return 0;
    }


    ///Aruco lists
    pugi::xml_node aruco = doc.child("arucoList");


    std::string readingValue;
    int id;
    double size;

    ArucoCodeDefinition ArucoAux;

    for(pugi::xml_node arucoMarker = aruco.child("arucoMarker");arucoMarker; arucoMarker = arucoMarker.next_sibling("arucoMarker"))
    {
        readingValue=arucoMarker.child_value("id");
        istringstream convertid(readingValue);
        convertid>>id;

        readingValue=arucoMarker.child_value("size");
        istringstream convertsize(readingValue);
        convertsize>>size;

        ArucoAux.setArucoCode(id,size);

        //Añadimos. TODO check that already doesnt exist
        if(!isCodeByIdInList(id))
        {
            ArucoListDefinitionCodes.push_back(ArucoAux);
        }
        else
        {
            noError=0;
        }
    }

    return noError;
}



bool ArucoListDefinition::isCodeByIdInList(int idCode)
{
    for(unsigned int i=0;i<ArucoListDefinitionCodes.size();i++)
    {
        if(ArucoListDefinitionCodes[i].getId()==idCode)
            return true;
    }

    return false;
}


int ArucoListDefinition::getCodeSizeById(double &sizeCode, int idCode)
{
    sizeCode=-1.0;

    for(unsigned int i=0;i<ArucoListDefinitionCodes.size();i++)
    {
        if(ArucoListDefinitionCodes[i].getId()==idCode)
        {
            sizeCode=ArucoListDefinitionCodes[i].getSize();
            return 1;
        }
    }
    return 0;
}

int ArucoListDefinition::getCodeSize(double &sizeCode, unsigned int codePosition)
{
    if(codePosition<ArucoListDefinitionCodes.size())
    {
        sizeCode=ArucoListDefinitionCodes[codePosition].getSize();
        return 1;
    }
    else
    {
        sizeCode=-1.0;
        return 0;
    }
    return 0;
}






//////////////// Aruco Eye /////////////////////////
ArucoEye::ArucoEye()
{
    init();
    return;
}

ArucoEye::~ArucoEye()
{
    close();
    return;
}

int ArucoEye::init()
{
    flagNewImage=false;

    if(!configureArucoDetector())
        return 0;

    return 1;
}

int ArucoEye::close()
{
    return 1;
}


int ArucoEye::configure(std::string arucoListFile, std::string cameraParametersFile)
{

    //Camera parameters
    if(!setCameraParameters(cameraParametersFile))
    {
        return 0;
    }

    //Aruco List
    if(!setArucoList(arucoListFile))
        return 0;


    //Aruco Detector
    if(!configureArucoDetector())
        return 0;



    return 1;
}


int ArucoEye::setCameraParameters(std::string filename)
{
    TheCameraParameters.readFromXMLFile(filename);
    return 1;
}


int ArucoEye::setArucoList(std::string arucoListFile)
{
    return ArucoList.loadListFromXmlFile(arucoListFile);
}


//Configure ArucoDetector
int ArucoEye::configureArucoDetector(bool enableErosion, aruco::MarkerDetector::ThresholdMethods thresholdMethod, double ThresParam1, double ThresParam2, aruco::MarkerDetector::CornerRefinementMethod methodCornerRefinement, int ThePyrDownLevel, float minSize, float maxSize)
{
    //Marker detector
    if(ThePyrDownLevel>0)
        MDetector.pyrDown(ThePyrDownLevel);


    //Threshold
    //Params
    //MDetector.getThresholdParams( ThresParam1,ThresParam2);
    MDetector.setThresholdParams(ThresParam1,ThresParam2);

    //Method
    //enum ThresholdMethods {FIXED_THRES,ADPT_THRES,CANNY};
    MDetector.setThresholdMethod(thresholdMethod);
    //ThresholdMethods = MDetector.getThresholdMethod();


    //Corner refinement method
    //enum CornerRefinementMethod {NONE,HARRIS,SUBPIX,LINES};
    MDetector.setCornerRefinementMethod(methodCornerRefinement);


    //Erosion for chesstable
    MDetector.enableErosion(enableErosion);


    //Specifies the min and max sizes of the markers as a fraction of the image size. By size we mean the maximum of cols and rows.
    //@param min size of the contour to consider a possible marker as valid (0,1]
    //@param max size of the contour to consider a possible marker as valid [0,1)
    MDetector.setMinMaxSize(minSize,maxSize);

    return 1;
}



int ArucoEye::run()
{
    //Checks
    if(!flagNewImage)
        return 0;

    if(InputImage.size().height==0 || InputImage.size().width==0)
        return 0;

    //Detection of markers in the image passed
    MDetector.detect(InputImage,TheMarkers);//,TheCameraParameters);

    //calculate 3d info
    if(TheCameraParameters.isValid())
    {
        for(int i=TheMarkers.size()-1;i>=0;i--)
        {
            double theMarkerSize;

            //Code is in the list
            if(ArucoList.getCodeSizeById(theMarkerSize,TheMarkers[i].id))
            {
                TheMarkers[i].calculateExtrinsics(theMarkerSize,TheCameraParameters);
            }
            else
            {
                //Code is not in the list. delete
                TheMarkers.erase(TheMarkers.begin()+i);
            }
        }
    }
    else
    {
#ifdef VERBOSE_ARUCO_EYE
        cout<<"[AE] Invalid camera parameters. Unable to reconstruct 3d"<<endl;
#endif
    }

    flagNewImage=false;

    return 1;
}

int ArucoEye::run(unsigned int &numCodesDetected, unsigned int &numCodesReconstructed)
{
    //Run
    if(!run())
        return 0;

    //Count
    numCodesDetected=0;
    numCodesReconstructed=0;

    //codes detected
    numCodesDetected=TheMarkers.size();

    //codes reconstructed
    if(TheCameraParameters.isValid())
    {
        numCodesReconstructed=0;
        for(unsigned int i=0;i<TheMarkers.size();i++)
        {
            //Code is in the list
            if(ArucoList.isCodeByIdInList(TheMarkers[i].id))
            {
                numCodesReconstructed++;
            }
        }
    }
    return 1;
}


char ArucoEye::drawDetectedArucoCodes(std::string windowName, int waitingTime, bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    //Checks
    if(InputImage.size().height==0 || InputImage.size().width==0)
        return '0';


    //////// Drawings //////

    //Copy
    InputImage.copyTo(OutputImage);


    //print marker info and draw the markers in image
    if(drawDetectedCodes)
    {
        for(unsigned int i=0;i<TheMarkers.size();i++)
        {
            TheMarkers[i].draw(OutputImage,cv::Scalar(0,0,255),true);
        }
    }


    //draw a 3d in each marker if there is 3d info
    if(draw3DReconstructedCodes)
    {
        if(TheCameraParameters.isValid())
        {
            for(unsigned int i=0;i<TheMarkers.size();i++)
            {
                //aruco::CvDrawingUtils::draw3dCube(OutputImage,TheMarkers[i],TheCameraParameters);
                aruco::CvDrawingUtils::draw3dAxis(OutputImage,TheMarkers[i],TheCameraParameters);
            }
        }
    }

    //show input with augmented information and  the thresholded image
    cv::imshow(windowName,OutputImage);

    //end
    return cv::waitKey(waitingTime);//wait for key to be pressed
}



int ArucoEye::setInputImage(cv::Mat InputImageIn)
{
    flagNewImage=true;
    InputImageIn.copyTo(InputImage);
    return 1;
}


int ArucoEye::getOutputImage(cv::Mat &OutputImageOut)
{
    OutputImage.copyTo(OutputImageOut);
    return 1;
}


int ArucoEye::getMarkers3DInformation(std::vector<aruco::Marker> &TheMarkers3D)
{
    TheMarkers3D=TheMarkers;
    return 1;
}


int ArucoEye::getMarkersI3DInformation(unsigned int markerI, int &idMarker, cv::Mat &RvecMarker, cv::Mat &TvecMarker)
{
    if(markerI>TheMarkers.size())
        return 0;

    idMarker=TheMarkers[markerI].id;
    TheMarkers[markerI].Rvec.copyTo(RvecMarker);
    TheMarkers[markerI].Tvec.copyTo(TvecMarker);

    return 1;
}






////////////////////// homogeneusTransformation ////////////////////////////
int homogeneusTransformation::createMatHomogFromVecs(cv::Mat& MatHomog, cv::Mat TransVec, cv::Mat RotVec)
{
	//MatHomog.create(Size(4,4),CV_64F);
    MatHomog=cv::Mat::zeros(4,4,CV_32F);
	
	
	//
    cv::Mat RotMat;
	cv::Rodrigues(RotVec,RotMat);
	
	//cout<<RotMat<<endl;
	
	
	//Rotations
	/*
	float roll=RotVec.at<float>(0,0);
	Mat MatHRoll=Mat::zeros(4,4,CV_32F); 	
	MatHRoll.at<float>(0,0)=1.0;
	MatHRoll.at<float>(1,1)=cos(roll); MatHRoll.at<float>(1,2)=-sin(roll);
	MatHRoll.at<float>(2,1)=sin(roll); MatHRoll.at<float>(2,2)=cos(roll);
	MatHRoll.at<float>(3,3)=1.0;
	float pitch=RotVec.at<float>(1,0);
	Mat MatHPitch=Mat::zeros(4,4,CV_32F); 
	MatHPitch.at<float>(0,0)=cos(pitch); MatHPitch.at<float>(0,2)=sin(pitch);
	MatHPitch.at<float>(1,1)=1.0;
	MatHPitch.at<float>(2,0)=-sin(pitch); MatHPitch.at<float>(2,2)=cos(pitch);
	MatHPitch.at<float>(3,3)=1.0;
	float yaw=RotVec.at<float>(2,0);
	Mat MatHYaw=Mat::zeros(4,4,CV_32F); 
	MatHYaw.at<float>(0,0)=cos(yaw); MatHYaw.at<float>(0,1)=-sin(yaw);
	MatHYaw.at<float>(1,0)=sin(yaw); MatHYaw.at<float>(1,1)=cos(yaw);
	MatHYaw.at<float>(2,2)=1.0;
	MatHPitch.at<float>(3,3)=1.0;
	*MatHomog=MatHRoll*MatHPitch*MatHYaw;
	*/
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
            MatHomog.at<float>(i,j)=RotMat.at<float>(i,j);
	
	
	
	//Translation
	for(int i=0;i<3;i++)
	{
        MatHomog.at<float>(i,3)=TransVec.at<float>(i,0);
	}
	
	
	//Scale
    MatHomog.at<float>(3,3)=1.0;
	
	
	//cout<<*MatHomog<<endl;
	
	return 1;
}



int homogeneusTransformation::calculateVecsFromMatHomog(cv::Mat& TransVec, cv::Mat& RotVec, cv::Mat MatHomog)
{
	
	//Translation
    TransVec=cv::Mat::zeros(3,1,CV_32F);
	for(int i=0;i<3;i++)
	{
        TransVec.at<float>(i,0)=MatHomog.at<float>(i,3);
	}
	
	
	//Rotations
    RotVec=cv::Mat::zeros(3,1,CV_32F);
	
	
	
	//Rotacion
    cv::Mat RotMat=cv::Mat::zeros(3,3,CV_32F);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			RotMat.at<float>(i,j)=MatHomog.at<float>(i,j);
	
    //cv::Mat AuxRotVec=*RotVec;
    cv::Rodrigues(RotMat,RotVec);
	

	return 1;
}



int homogeneusTransformation::calculateTraVecYPRFromMatHomog(cv::Mat *YPRVec, cv::Mat *TraVec, cv::Mat MatHomog)
{
    //Translation
    *TraVec=cv::Mat::zeros(3,1,CV_32F);
    for(int i=0;i<3;i++)
    {
        TraVec->at<float>(i,0)=MatHomog.at<float>(i,3);
    }



    //Rotation
    *YPRVec=cv::Mat::zeros(3,1,CV_32F);
	


	float yaw, pitch, roll;


    /*
    ///http://www.euclideanspace.com/maths/geometry/rotations/euler/indexLocal.htm
    ///http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm

    pitch=asin(MatHomog.at<float>(1,0));
    if(MatHomog.at<float>(1,0)==1) //North pole
    {
        yaw=atan2(MatHomog.at<float>(0,2),MatHomog.at<float>(2,2));
        roll=0.0;
    }
    else if(MatHomog.at<float>(1,0)==-1) //South pole
    {
        yaw=atan2(MatHomog.at<float>(0,2),MatHomog.at<float>(2,2));
        roll=0.0;
    }
    else
    {
        yaw=atan2(-MatHomog.at<float>(1,2),MatHomog.at<float>(1,1));
        roll=atan2(-MatHomog.at<float>(2,0),MatHomog.at<float>(0,0));
    }


    //end
    YPRVec->at<float>(0,0)=yaw;
    YPRVec->at<float>(1,0)=pitch;
    YPRVec->at<float>(2,0)=roll;


    return 1;
    */





    /*
    ////http://www.mathworks.es/matlabcentral/fileexchange/24589-kinematics-toolbox/content/kinematics/screws/rpy.m
    //Pitch
    float tol=1e-4;
    if(pow(MatHomog.at<float>(0,0),2) + pow(MatHomog.at<float>(0,1),2) >=tol)
    {
        pitch=atan2(MatHomog.at<float>(0,2), sqrt(pow(MatHomog.at<float>(0,0),2) + pow(MatHomog.at<float>(0,1),2))); // mal condicionada si está cerca de pi/2 o -pi/2
    }
    else
    {
        cout<<"!Mal condicionada pitch\n";
        pitch=asin(MatHomog.at<float>(0,2));
    }


    //Yaw and Roll
    float tolPitch=0.0;
    if(pitch>=PI/2-tolPitch)
    {
        //North pole
        cout<<"!Mal condicionada pitch a\n";
        roll = 0;
        yaw = atan2(MatHomog.at<float>(0,1), MatHomog.at<float>(1,1));
    }
    else if(pitch<=-PI/2+tolPitch)
    {
        //South pole
        cout<<"!Mal condicionada pitch b\n";
        roll = 0;
        yaw = -atan2(MatHomog.at<float>(0,1), MatHomog.at<float>(1,1));
    }
    else
    {
        //yaw = atan2(MatHomog.at<float>(1,0)/cos(pitch), MatHomog.at<float>(0,0)/cos(pitch));
        yaw=atan2(-MatHomog.at<float>(0,1),MatHomog.at<float>(0,0));
        //roll = atan2(MatHomog.at<float>(2,1)/cos(pitch), MatHomog.at<float>(2,2)/cos(pitch));
        roll=atan2(-MatHomog.at<float>(1,2),MatHomog.at<float>(2,2));
    }


    //
    YPRVec->at<float>(0,0)=yaw;
    YPRVec->at<float>(1,0)=pitch;
    YPRVec->at<float>(2,0)=roll;


    return 1;
    */



    ///http://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation
    float tol=1e-4;
	
	//Pitch
    if(pow(MatHomog.at<float>(0,0),2) + pow(MatHomog.at<float>(0,1),2) >=tol)
	{
	  float my_sign = MatHomog.at<float>(0,0) > 0 ? 1.0 : -1.0;
	  pitch=atan2(MatHomog.at<float>(0,2), my_sign*sqrt(pow(MatHomog.at<float>(0,0),2) + pow(MatHomog.at<float>(0,1),2))); // mal condicionada si está cerca de pi/2 o -pi/2


	//	if (MatHomog.at<float>(0,2) < 0 && MatHomog.at<float>(0,2) < 0 && MatHomog.at<float>(0,1))
	
	  /*
	cout << "YPR Calculation ........... GOOD Pitch!!!!!!";
	cout <<  " 0,2= " << MatHomog.at<float>(0,2);
	cout <<  " 0,0= " << MatHomog.at<float>(0,0);
	cout <<  " 0,1= " << MatHomog.at<float>(0,1) << endl;
	  */
	}
	else
	{
        //TODO
      cout<<"!Bad conditioning in Pitch. Pitch =+-pi/2" << endl;
      pitch = cvg_utils_library::asin_ws(MatHomog.at<float>(0,2));
	}
	
	

    float tol2=-1e-1;

	//Roll =f(pitch) -> pitch=+-pi/2 || yaw=+-pi/2
    if(abs(MatHomog.at<float>(2,2))<=tol2)
	{
	  cout<<"!Bad conditioning in Roll. Roll =+-pi/2" << endl;
        //TODO
		roll=0.0;
	}
	else
	{
		roll=atan2(-MatHomog.at<float>(1,2),MatHomog.at<float>(2,2));
	}
	
	
	//Yaw =f(roll)
    if(abs(MatHomog.at<float>(0,0))<=tol2)
	{
	  cout<<"!Bad conditioning in Yaw. Yaw =+-pi/2" << endl;
        //TODO
      float aux = (cos(roll)-MatHomog.at<float>(1,1)/MatHomog.at<float>(2,1)*sin(roll)) / (MatHomog.at<float>(1,0)-MatHomog.at<float>(1,1)*MatHomog.at<float>(2,0)/MatHomog.at<float>(2,1));
      yaw = cvg_utils_library::asin_ws(aux);
	}
	else
	{
		yaw=atan2(-MatHomog.at<float>(0,1),MatHomog.at<float>(0,0));
	}
	

	//end
	YPRVec->at<float>(0,0)=yaw;
	YPRVec->at<float>(1,0)=pitch;
	YPRVec->at<float>(2,0)=roll;
	
	
    return 1;

}


int homogeneusTransformation::calculateMatHomogFromYPRVecTraVec(cv::Mat* MatHomog, cv::Mat YPRVec, cv::Mat TraVec)
{
    *MatHomog=cv::Mat::zeros(4,4,CV_32F);
	
	
	//Rotations
	
	float roll=YPRVec.at<float>(2,0);
    cv::Mat MatHRoll=cv::Mat::zeros(4,4,CV_32F);
	MatHRoll.at<float>(0,0)=1.0;
	MatHRoll.at<float>(1,1)=cos(roll); MatHRoll.at<float>(1,2)=-sin(roll);
	MatHRoll.at<float>(2,1)=sin(roll); MatHRoll.at<float>(2,2)=cos(roll);
	MatHRoll.at<float>(3,3)=1.0;
	float pitch=YPRVec.at<float>(1,0);
    cv::Mat MatHPitch=cv::Mat::zeros(4,4,CV_32F);
	MatHPitch.at<float>(0,0)=cos(pitch); MatHPitch.at<float>(0,2)=sin(pitch);
	MatHPitch.at<float>(1,1)=1.0;
	MatHPitch.at<float>(2,0)=-sin(pitch); MatHPitch.at<float>(2,2)=cos(pitch);
	MatHPitch.at<float>(3,3)=1.0;
	float yaw=YPRVec.at<float>(0,0);
    cv::Mat MatHYaw=cv::Mat::zeros(4,4,CV_32F);
	MatHYaw.at<float>(0,0)=cos(yaw); MatHYaw.at<float>(0,1)=-sin(yaw);
	MatHYaw.at<float>(1,0)=sin(yaw); MatHYaw.at<float>(1,1)=cos(yaw);
	MatHYaw.at<float>(2,2)=1.0;
	MatHYaw.at<float>(3,3)=1.0;
	*MatHomog=MatHRoll*MatHPitch*MatHYaw;
	
	/*
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			MatHomog->at<float>(i,j)=RotMat.at<float>(i,j);
	*/
	
	
	//Translation
	for(int i=0;i<3;i++)
	{
        MatHomog->at<float>(i,3)=TraVec.at<float>(i,0);
	}
	
	
	//Scale
	MatHomog->at<float>(3,3)=1.0;
	
	
	return 1;
}



int calculateMatHomogFromRotMatTraVec(cv::Mat* MatHomog, cv::Mat RotMat, cv::Mat TraVec)
{
    *MatHomog=cv::Mat::zeros(4,4,CV_32F);

    //Rotation
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            MatHomog->at<float>(i,j)=RotMat.at<float>(i,j);


    //Translation
    for(int i=0;i<3;i++)
    {
        MatHomog->at<float>(i,3)=TraVec.at<float>(i,0);
    }


    //Scale
    MatHomog->at<float>(3,3)=1.0;

    return 1;
}


