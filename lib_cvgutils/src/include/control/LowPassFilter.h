/*
 * LowPassFilter.h
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

#include "Timer.h"
#include <math.h>

namespace CVG_BlockDiagram {

class LowPassFilter {
private:
	// Input/output
    double u_k; // input
    double y_k; // output

	// Parameters
    double tr;	// response time
    double max_output;
    double min_output;
    double max_speed;

	// Internal state
    double y_km1; // last output
    Timer timer;
    bool started, saturation_enabled;

public:
	LowPassFilter();
	virtual ~LowPassFilter();

	void reset();

    inline void setResponseTime(double tr) { if (tr > 0) this->tr = tr; else this->tr = 1.0; }
    void enableSaturation(bool enable, double min, double max);
    void enableMaxSpeedSaturation(double max_speed);

    inline void setInternalyk(double y_kint) { this->y_k = y_kint; }
    inline void setInput(double u_k) 	{ this->u_k = u_k; }
    inline double getInput() 			{ return u_k; }
    double getOutput(bool use_last_input = false);
};

} /* namespace CVG_BlockDiagram */
#endif /* LOWPASSFILTER_H_ */
