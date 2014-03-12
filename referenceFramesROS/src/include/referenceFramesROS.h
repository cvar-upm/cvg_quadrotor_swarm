/*
 * referenceFrames.h
 *
 *  Created on: 
 *      Author: jl.sanchez
 */

#ifndef REFERENCE_FRAMES_ROS_H
#define REFERENCE_FRAMES_ROS_H



////// ROS  ///////
#include "ros/ros.h"




//Cpp
#include <sstream>
#include <string>
#include <iostream>




//Messages
#include "droneMsgsROS/dronePose.h"

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/opencv.hpp>

#include "cvg_utils_library.h"

namespace referenceFrames
{

//Paloma 2 Sist ref global
int refFrameChangeXYZ2ZYX(droneMsgsROS::dronePose pose_xYyPzR, droneMsgsROS::dronePose* pose_zYyPxR); //mobile robots 2 common
int refFrameChangeZYX2XYZ(droneMsgsROS::dronePose pose_zYyPxR, droneMsgsROS::dronePose* pose_xYyPzR); //common 2 mobile robots

// xYyPzR 2 wYvPuR 
int xYyPzR2wYvPuR(droneMsgsROS::dronePose pose_xYyPzR, droneMsgsROS::dronePose* pose_wYvPuR); // GMR xYyPzR 2 GMR wYvPuR
// wYvPuR 2 xYyPzR
int wYvPuR2xYyPzR(droneMsgsROS::dronePose pose_wYvPuR, droneMsgsROS::dronePose* pose_xYyPzR); // GMR wYvPuR 2 GMR xYyPzR (mine_p)



}
        
        
 

#endif
