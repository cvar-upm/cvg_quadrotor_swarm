/*
 * cvg_utils_library.h
 *
 *  Created on: Apr 25, 2012
 *      Author: jespestana
 */

#ifndef CVG_UTILS_LIBRARY_H_
#define CVG_UTILS_LIBRARY_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "matrixLib.h"
#include <stdexcept>


namespace cvg_utils_library {
    double interpolate(double x, CVG::Vector *x_val, CVG::Vector *y_val);
	double saturate(double x, double x_lim_inf, double x_lim_sup);
//	double mapAngleToMinusPlusPi(double angleRads);
//	std::string convertInt(int number);

	// cambia angleRads2Move si ambos angulos son fisicamente cercanos pero caen uno por un lado de -pi y el otro por el otro lado de pi
	double mapAnglesToBeNear_PIrads(double angleRads2move, double angleRads);


	// Vector related functions
    void flipVector( CVG::Vector &v, int sign);
	int fmod(int numerator, int denominator);
//	void unitaryVectorFrom2Points( CVG::Vector &ur, CVG::Vector &p0, CVG::Vector &p1);
    double unitaryVectorFrom2Points( CVG::Vector &ur, CVG::Vector &p0, CVG::Vector &p1); // from p0 to p1
    double normOfVector( CVG::Vector &r_p);
    void getVectorComponents( CVG::Vector &u, double &ux, double &uy, double &uz);
    void setVectorComponents( CVG::Vector &u, double &ux, double &uy, double &uz);
    double dotProduct( CVG::Vector &u, CVG::Vector&v);
    void crossProduct( CVG::Vector &result, CVG::Vector &u, CVG::Vector &v);
    void unitarizeVector( CVG::Vector &u);
    void multiplyDoubleVsVector( double c, CVG::Vector &u);
    double distanceAlongLine( CVG::Vector &p0, CVG::Vector &p, CVG::Vector &r_ur);
    double calculateVmax( CVG::Vector &r_ur, double vmax_xy, double vmax_z);
    void mostrarVector( std::string nombreVector, CVG::Vector &u);

    // yaws, yawci problem in yaw pid controller (problem with angle ranges and so on), I suppose I got yawci - yaws
    double /* double yaw_error_rad = yawci - yaws */ getAngleError(double yawci_rad, double yaws_rad); // error in range [-M_PI, +M_PI]
    double dotProduct2D( double ux, double uy, double vx, double vy);
    double crossProduct2D( double ux, double uy, double vx, double vy);

    double asin_ws(double sin_val);
    double acos_ws(double cos_val);
}

#endif /* CVG_UTILS_LIBRARY_H_ */
