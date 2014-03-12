/*
 * PID.h
 *
 *  Created on: 19/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef PID_H_
#define PID_H_

#include "Timer.h"

#define PID_D_FILTER_LENGTH	5

namespace CVG_BlockDiagram {

class PID {
private:
	// Input/output
    double reference;
    double feedback;
    double output;

	// Parameters
    double Kp, Ki, Kd;
    double actuatorSaturation;
    double maxOutput;

	// Internal state
    double integrator;
    double lastError;
    double dErrorHistory[PID_D_FILTER_LENGTH];
	Timer timer;
    bool started;

public:
	PID();
	virtual ~PID();

    inline void setGains(const double p, const double i, const double d) { Kp = p; Ki = i; Kd = d; }
    inline void getGains(double &p, double &i, double &d) { p = Kp; i = Ki; d = Kd; }
    void enableMaxOutput(bool enable, double max);
    void enableAntiWindup(bool enable, double actuatorSaturation);

    inline void setReference(double ref) { reference = ref; }
    inline void setFeedback(double measure) { feedback = measure; }
    double getOutput();

	void reset();
};

} /* namespace CVG_BlockDiagram */
#endif /* PID_H_ */
