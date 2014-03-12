#pragma once

#define  NOMINMAX 

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH      
#include "newmatap.h"                // need matrix applications
#include "newmatio.h"                // need matrix output routines
#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif  

//#define PI 3.14159265
//#define SGN(x) ((x)>0?1:-1)
//#define DEG2RAD 0.017452777
//#define RAD2DEG 57.29747191


#include "htrans.h"
#include <vector>

float AngRango (float ang);

ColumnVector Comp(ColumnVector ab, ColumnVector bc);
ColumnVector CompOdom(ColumnVector ab, ColumnVector bc);

ColumnVector Inv(ColumnVector ab);

Matrix GetCoord (const HomogTrans& h);

Matrix J1(ColumnVector ab,ColumnVector bc, ColumnVector ac);
Matrix J1(ColumnVector ab,ColumnVector bc);
Matrix J1_n(ColumnVector ab,ColumnVector bc);

Matrix J1Odom(ColumnVector ab,ColumnVector bc);
Matrix J1_M (ColumnVector ab,ColumnVector bc);   

Matrix J2(ColumnVector ab, ColumnVector bc, ColumnVector ac);
Matrix J2(ColumnVector ab, ColumnVector bc);
Matrix J2_n(ColumnVector ab, ColumnVector bc);

Matrix J2Odom(ColumnVector ab, ColumnVector bc);
Matrix J2_A (ColumnVector ab,ColumnVector bc);

Matrix JInv(ColumnVector v);

Matrix InvTrans(Matrix x_ab);
