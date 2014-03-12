/*
 * FilteredDerivative.h
 *
 *  Created on: Dec 14, 2012
 *      Author: jespestana
 */

#ifndef FILTEREDDERIVATIVE_H_
#define FILTEREDDERIVATIVE_H_

#include "Timer.h"
#include "control/LowPassFilter.h"

namespace CVG_BlockDiagram {

class FilteredDerivative {
private:
	// Input/output
    double x_k; // input
    double dx_k; // output

	// Parameters
	LowPassFilter lowpassfilter;
	Timer timer;

	// Internal state
    double x_km1; // last output
    bool started;

public:
	FilteredDerivative();
	virtual ~FilteredDerivative();

	void reset();

    inline void setResponseTime(double tr) { lowpassfilter.setResponseTime(tr); }
    void enableSaturation(bool enable, double min, double max);

    inline void setInternaldxk(double dx_kint) { lowpassfilter.setInternalyk( dx_kint); }
    inline void setInput(double x_k) 	{ this->x_k = x_k; }
    inline double getInput() 			{ return x_k; }
    double getOutput(bool use_last_input = false);
};

}

#endif /* FILTEREDDERIVATIVE_H_ */
