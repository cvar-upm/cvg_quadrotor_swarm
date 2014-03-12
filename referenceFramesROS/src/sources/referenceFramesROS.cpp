/*
 * referenceFrames.cpp
 *
 *  Created on: 
 *      Author: jl.sanchez
 */


#include "referenceFramesROS.h"
#include "htrans.h"
#include <cmath>
#include <ctgmath>

namespace referenceFrames
{
int refFrameChangeXYZ2ZYX(droneMsgsROS::dronePose pose_xYyPzR, droneMsgsROS::dronePose* pose_zYyPxR) //mobile robots 2 common
{  
    float x_input = pose_xYyPzR.x;
    float y_input = pose_xYyPzR.y;
    float z_input = pose_xYyPzR.z;
    float roll_input = pose_xYyPzR.roll;
    float pitch_input = pose_xYyPzR.pitch;
    float yaw_input = pose_xYyPzR.yaw;
    
    HomogTrans h;
    h.DirectTrans(x_input,y_input,z_input,roll_input,pitch_input,yaw_input);
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = atan2(-h.mat[1][2],h.mat[2][2]);
    float pitch_output = cvg_utils_library::asin_ws(h.mat[0][2]);
    float yaw_output = cvg_utils_library::acos_ws(h.mat[0][0]/cos(pitch_output));
    
    pose_zYyPxR->x = x_output;
    pose_zYyPxR->y = y_output;
    pose_zYyPxR->z = z_output;
    pose_zYyPxR->roll = roll_output;
    pose_zYyPxR->pitch = pitch_output;
    pose_zYyPxR->yaw = yaw_output;
    
    
    return 1;
}

int refFrameChangeZYX2XYZ(droneMsgsROS::dronePose pose_zYyPxR, droneMsgsROS::dronePose* pose_xYyPzR) //common 2 mobile robots
{
    
    float x_input = pose_zYyPxR.x;
    float y_input = pose_zYyPxR.y;
    float z_input = pose_zYyPxR.z;
    float roll_input = pose_zYyPxR.roll;
    float pitch_input = pose_zYyPxR.pitch;
    float yaw_input = pose_zYyPxR.yaw;
    
    HomogTrans h;
    h.DirectTrans(x_input,y_input,z_input,roll_input,pitch_input,yaw_input);
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = atan2(h.mat[1][0],h.mat[0][0]);
    float pitch_output = atan2(-h.mat[2][0],cos(roll_output)*h.mat[0][0]+sin(roll_output)*h.mat[1][0]);
    float yaw_output = atan2(sin(roll_output)*h.mat[0][2]-cos(roll_output)*h.mat[1][2], -sin(roll_output)*h.mat[0][1]+cos(roll_output)*h.mat[1][1]);
    
    pose_xYyPzR->x = x_output;
    pose_xYyPzR->y = y_output;
    pose_xYyPzR->z = z_output;
    pose_xYyPzR->roll = roll_output;
    pose_xYyPzR->pitch = pitch_output;
    pose_xYyPzR->yaw = yaw_output;    
    
    return 1;
}

// xYyPzR 2 wYvPuR 2
int xYyPzR2wYvPuR(droneMsgsROS::dronePose pose_xYyPzR, droneMsgsROS::dronePose* pose_wYvPuR) // GMR 2 LMrT
{
	 float x_input = pose_xYyPzR.x;
    float y_input = pose_xYyPzR.y;
    float z_input = pose_xYyPzR.z;
    float roll_input = pose_xYyPzR.roll;
    float pitch_input = pose_xYyPzR.pitch;
    float yaw_input = pose_xYyPzR.yaw;
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = yaw_input;
    float pitch_output = pitch_input;
    float yaw_output = roll_input;
    
    pose_wYvPuR->x = x_output;
    pose_wYvPuR->y = y_output;
    pose_wYvPuR->z = z_output;
    pose_wYvPuR->roll = roll_output;
    pose_wYvPuR->pitch = pitch_output;
    pose_wYvPuR->yaw = yaw_output;
    

    return 1;



}
// wYvPuR 2 xYyPzR
int wYvPuR2xYyPzR(droneMsgsROS::dronePose pose_wYvPuR, droneMsgsROS::dronePose* pose_xYyPzR) // LMrT 2 GMR
{

	 float x_input = pose_wYvPuR.x;
    float y_input = pose_wYvPuR.y;
    float z_input = pose_wYvPuR.z;
    float roll_input = pose_wYvPuR.roll;
    float pitch_input = pose_wYvPuR.pitch;
    float yaw_input = pose_wYvPuR.yaw;
    
    float x_output = x_input;
    float y_output = y_input;
    float z_output = z_input;
    float roll_output = yaw_input;
    float pitch_output = pitch_input;
    float yaw_output = roll_input;
    
    pose_xYyPzR->x = x_output;
    pose_xYyPzR->y = y_output;
    pose_xYyPzR->z = z_output;
    pose_xYyPzR->roll = roll_output;
    pose_xYyPzR->pitch = pitch_output;
    pose_xYyPzR->yaw = yaw_output;

    
    return 1;



}



}







