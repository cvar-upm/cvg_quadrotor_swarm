/*
 * MidLevelControlledsimMultirotor.cpp
 *
 *  Created on: Dec 17, 2012
 *      Author: jespestana
 */

#include "control/MidLevelControlledsimMultirotor.h"

namespace CVG_BlockDiagram {

MidLevel_Controlled_simMultirotor::MidLevel_Controlled_simMultirotor( double x0, double y0, double z0) {
	vmax_at_xy = SIMMULTIROTOR_TRAJECTORYCONTROLLER_VXY_AT_MAX;
	vmax_ct_xy = SIMMULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX;
	vmax_at_z  = SIMMULTIROTOR_TRAJECTORYCONTROLLER_VZ_AT_MAX;
	vmax_ct_z  = SIMMULTIROTOR_TRAJECTORYCONTROLLER_VZ_CT_MAX;

	xc = 0.0, yc = 0.0, zc = 0.0, vxc = 0.0, vyc = 0.0, vzc = 0.0;
	xo = x0, yo = y0, zo = z0, vxo = 0.0, vyo = 0.0, vzo = 0.0;
	vx_aux = 0.0, vy_aux = 0.0, vz_aux = 0.0;

	tr_z  = SIMMULTIROTOR_SPEEDCONTROLLER_Z_TR;
	tr_vz = SIMMULTIROTOR_SPEEDCONTROLLER_DZ_TR;
	tr_xy = SIMMULTIROTOR_SPEEDCONTROLLER_XY_TR;
	tr_vxy= SIMMULTIROTOR_SPEEDCONTROLLER_VXY_TR;

	// position filters
	xfilter0.setResponseTime( tr_xy );
	xfilter0.reset();
	xfilter0.enableMaxSpeedSaturation(vmax_ct_xy);

	yfilter0.setResponseTime( tr_xy );
	yfilter0.reset();
	yfilter0.enableMaxSpeedSaturation(vmax_ct_xy);

	zfilter0.setResponseTime( tr_z );
	zfilter0.reset();
	zfilter0.enableMaxSpeedSaturation(vmax_ct_z);

	// tangent speed filter
	dxfilter0.setResponseTime(tr_vxy);
	dxfilter0.reset();
	dxfilter0.enableSaturation( true, -vmax_at_xy, vmax_at_xy);
	dyfilter0.setResponseTime(tr_vxy);
	dyfilter0.reset();
	dyfilter0.enableSaturation( true, -vmax_at_xy, vmax_at_xy);
	dzfilter0.setResponseTime(tr_vz);
	dzfilter0.reset();
	dzfilter0.enableSaturation( true, -vmax_at_z, vmax_at_z);

	// speed output filter
	dxofilter0.setResponseTime( tr_vxy/3.0);
	dxofilter0.reset();
	dyofilter0.setResponseTime( tr_vxy/3.0);
	dyofilter0.reset();
	dzofilter0.setResponseTime( tr_vz/3.0);
	dzofilter0.reset();

	// other resources
	started = false;
	timer.restart(false);
}


MidLevel_Controlled_simMultirotor::~MidLevel_Controlled_simMultirotor() {
}

void MidLevel_Controlled_simMultirotor::reset() {
	started = false;
	timer.restart(false);
}

void MidLevel_Controlled_simMultirotor::reset( double x0, double y0, double z0) {
	reset();
	xc = x0, yc = y0, zc = z0;
	xo = x0, yo = y0, zo = z0;
}

void MidLevel_Controlled_simMultirotor::actualReset( double x0, double y0, double z0) {
	xc = x0, yc = y0, zc = z0, vxc = 0.0, vyc = 0.0, vzc = 0.0;
	xo = x0, yo = y0, zo = z0, vxo = 0.0, vyo = 0.0, vzo = 0.0;
	vx_aux = 0.0, vy_aux = 0.0, vz_aux = 0.0;

	// position filters
	xfilter0.reset();
	yfilter0.reset();
	zfilter0.reset();

	// tangent speed filter
	dxfilter0.reset();
	dyfilter0.reset();
	dzfilter0.reset();

	// speed output filter
	dxofilter0.reset();
	dyofilter0.reset();
	dzofilter0.reset();

	// other resources
	timer.restart(false);
	started = true;

	getOutput(xo, yo, zo, vxo, vyo, vzo);
}

void MidLevel_Controlled_simMultirotor::setInput( double xc, double yc, double zc, double vxc, double vyc, double vzc) {
	this->xc = xc;
	this->yc = yc;
	this->zc = zc;
	this->vxc = vxc;
	this->vyc = vyc;
	this->vzc = vzc;
}

void MidLevel_Controlled_simMultirotor::getOutput(double& xo_o, double& yo_o, double& zo_o, double& vxo_o, double& vyo_o, double& vzo_o) {

    double elapsed = timer.getElapsedSeconds();
	timer.restart(started);
	if (!started) { // start sequence
		actualReset( xc, yc, zc);
		started = true;
	} else {

		// calculate output
		// calculate cross-track position movement
		xfilter0.setInternalyk( xo);
		xfilter0.setInput( xc);
		xo = xfilter0.getOutput();

		yfilter0.setInternalyk( yo);
		yfilter0.setInput( yc);
		yo = yfilter0.getOutput();

		zfilter0.setInternalyk( zo);
		zfilter0.setInput( zc);
		zo = zfilter0.getOutput();

		// calculate along-track position filter
//		dxfilter0.setInternalyk( vx_aux);
		dxfilter0.setInput( vxc);
		vx_aux = dxfilter0.getOutput();

//		dyfilter0.setInternalyk( vy_aux);
		dyfilter0.setInput( vyc);
		vy_aux = dyfilter0.getOutput();

//		dzfilter0.setInternalyk( vz_aux);
		dzfilter0.setInput( vzc);
		vz_aux = dzfilter0.getOutput();

		xo += vx_aux*elapsed;
		yo += vy_aux*elapsed;
		zo += vz_aux*elapsed;

		// speed output filter
		dxofilter0.setInput( xo);
		vxo = dxofilter0.getOutput();
		dyofilter0.setInput( yo);
		vyo = dyofilter0.getOutput();
		dzofilter0.setInput( zo);
		vzo = dzofilter0.getOutput();
	}

	xo_o  = xo;
	yo_o  = yo;
	zo_o  = zo;
	vxo_o = vxo;
	vyo_o = vyo;
	vzo_o = vzo;
	return;
}

} /* namespace CVG_BlockDiagram */
