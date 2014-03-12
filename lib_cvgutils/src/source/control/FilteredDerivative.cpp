/*
 * FilteredDerivative.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: jespestana
 */

#include "control/FilteredDerivative.h"

namespace CVG_BlockDiagram {

FilteredDerivative::FilteredDerivative() : lowpassfilter() {
	x_k  = 0; 	// input
	dx_k = 0.0; // output

	lowpassfilter.setResponseTime( 1.0);
	lowpassfilter.reset();

	x_km1   = 0.0; // last output
	started = false;

	timer.restart(false);
}

FilteredDerivative::~FilteredDerivative() {
}

void FilteredDerivative::reset() {
	lowpassfilter.reset();
	started = false;
	timer.restart(false);
}

void FilteredDerivative::enableSaturation(bool enable, double min, double max) {
	lowpassfilter.enableSaturation( enable, min, max);
}

double FilteredDerivative::getOutput(bool use_last_input) {

    double elapsed = timer.getElapsedSeconds();
	timer.restart(started);
	if (!started) { // start sequence
//		x_k   = x_k;
		x_km1 = x_k;
		lowpassfilter.setInput( 0.0);
		dx_k  = lowpassfilter.getOutput();
//		dx_k  = 0.0;

		started = true;
		return dx_k;
	}

	// calculate output
	lowpassfilter.setInput( (x_k - x_km1)/elapsed );
	dx_k = lowpassfilter.getOutput();
	// saturation calculated inside lowpassfilter.getOutput();

	x_km1 = x_k;
	return dx_k;
}

} // namespace CVG_BlockDiagram



