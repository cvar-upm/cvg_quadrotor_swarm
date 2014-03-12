/*
 * MidLevelControlledsimMultirotor.h
 *
 *  Created on: Dec 17, 2012
 *      Author: jespestana
 */

#ifndef MIDLEVELCONTROLLEDSIMMULTIROTOR_H_
#define MIDLEVELCONTROLLEDSIMMULTIROTOR_H_

#include "Timer.h"
#include "control/LowPassFilter.h"
#include "control/FilteredDerivative.h"

// ***** Simulator configuration parameters *****
#define SIMMULTIROTOR_SPEEDCONTROLLER_DZMAX 			0.6	 // m/s
#define SIMMULTIROTOR_SPEEDCONTROLLER_VXY_MAX 			2.50 // m/s
#define SIMMULTIROTOR_TRAJECTORYCONTROLLER_VXY_AT_MAX 	((    0.75) * SIMMULTIROTOR_SPEEDCONTROLLER_VXY_MAX ) // m/s
#define SIMMULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX 	((1 - 0.75) * SIMMULTIROTOR_SPEEDCONTROLLER_VXY_MAX ) // m/s
#define SIMMULTIROTOR_TRAJECTORYCONTROLLER_VZ_AT_MAX 	((    0.75) * SIMMULTIROTOR_SPEEDCONTROLLER_DZMAX)  // m/s
#define SIMMULTIROTOR_TRAJECTORYCONTROLLER_VZ_CT_MAX 	((1 - 0.75) * SIMMULTIROTOR_SPEEDCONTROLLER_DZMAX)  // m/s

#define SIMMULTIROTOR_SPEEDCONTROLLER_Z_TR 				9.0  // seg
#define SIMMULTIROTOR_SPEEDCONTROLLER_DZ_TR 			3.0  // seg
#define SIMMULTIROTOR_SPEEDCONTROLLER_XY_TR 			6.0  // seg
#define SIMMULTIROTOR_SPEEDCONTROLLER_VXY_TR 			2.0  // seg
// END: ***** Simulator configuration parameters *****

namespace CVG_BlockDiagram {

class MidLevel_Controlled_simMultirotor {
private:
	// configuration parameters
    double vmax_at_xy, vmax_ct_xy, vmax_at_z, vmax_ct_z;
    double tr_xy, tr_vxy, tr_z, tr_vz;

	// input variables
    double xc, yc, zc, vxc, vyc, vzc;
	// output variables
    double xo, yo, zo, vxo, vyo, vzo;
	// auxiliary variables
    double vx_aux, vy_aux, vz_aux;

	// position filters
    CVG_BlockDiagram::LowPassFilter xfilter0;
	CVG_BlockDiagram::LowPassFilter yfilter0;
	CVG_BlockDiagram::LowPassFilter zfilter0;
	// tangent speed filter
	CVG_BlockDiagram::LowPassFilter dxfilter0;
	CVG_BlockDiagram::LowPassFilter dyfilter0;
	CVG_BlockDiagram::LowPassFilter dzfilter0;
	// speed output filter
	CVG_BlockDiagram::FilteredDerivative dxofilter0;
	CVG_BlockDiagram::FilteredDerivative dyofilter0;
	CVG_BlockDiagram::FilteredDerivative dzofilter0;

	// other resources
    bool started;
	Timer timer;
public:
    MidLevel_Controlled_simMultirotor( double x0, double y0, double z0);
	virtual ~MidLevel_Controlled_simMultirotor();

public:  void reset();
         void reset( double x0, double y0, double z0);
private: void actualReset( double x0, double y0, double z0);

public:
    void setInput( double xc, double yc, double zc, double vxc, double vyc, double vzc);
    void getOutput(double& xo, double& yo, double& zo, double& vxo, double& vyo, double& vzo);

	// for safety zone tests
    inline void setPosition( double xnew, double ynew, double znew) 	{ actualReset( xnew, ynew, znew); }
};

} /* namespace CVG_BlockDiagram */
#endif /* MIDLEVELCONTROLLEDSIMMULTIROTOR_H_ */
