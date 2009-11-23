/** <b> Implementation file: do not include.\ </b> Defines barrett::Wam.
 *
 * @file wam-inl.h
 * @date Sep 25, 2009
 * @author Dan Cody
 *
 * @warning
 * This file is located in a \c detail directory. It is part of the
 * implementation and should not be directly included by the user.
 * @see barrett/wam.h
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */


#include <iostream>
#include <stdexcept>
#include <map>

#include <sys/mman.h>
#include <syslog.h>

#include <gsl/gsl_vector.h>

#include <barrett/wam/wam.h>
#include <barrett/wam/wam_local.h>

#include "../units.h"
#include "../systems/abstract/system.h"


namespace barrett {

// TODO(dc): some of these members should be inline

template<size_t DOF>
Wam<DOF>::Wam() :
	input(this), jpOutput(&jpOutputValue), jvOutput(&jvOutputValue),
	operateCount(), wam(NULL), wamLocal(NULL)
{
	// initialize syslog
	openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

	// lock memory
	mlockall(MCL_CURRENT | MCL_FUTURE);

	// open the WAM
	bt_wam_create(&wam, "wamg");
	// TODO(dc): verify that the DOF matches
	if (wam == NULL) {
		// TODO(dc): better exception, add throw declaration to function def
		throw std::runtime_error(
				"(Wam::Wam()): Couldn't make WAM. "
				"Check /var/log/syslog for more info.");
	}

	wamLocal = bt_wam_get_local(wam);

	// register the Wam, then register the callback
	activeWams[wamLocal] = this;
	bt_wam_local_set_callback(wamLocal, &Wam::handleCallback);
}

template<size_t DOF>
Wam<DOF>::~Wam()
{
	// stop receiving callbacks, then unregister the Wam
	bt_wam_local_set_callback(wamLocal, NULL);
	activeWams.erase(wamLocal);

	bt_wam_destroy(wam);
}


template<size_t DOF>
units::JointPositions<DOF> Wam<DOF>::getJointPositions()
{
	jp_type jp;

	// TODO(dc): make conversion to/from GSL vectors
	for (size_t i = 0; i < DOF; ++i) {
		jp[i] = gsl_vector_get(wamLocal->jposition, i);
	}

	return jp;
}

template<size_t DOF>
units::JointVelocities<DOF> Wam<DOF>::getJointVelocities()
{
	jv_type jv;

	// TODO(dc): make conversion to/from GSL vectors
	for (size_t i = 0; i < DOF; ++i) {
		jv[i] = gsl_vector_get(wamLocal->jvelocity, i);
	}

	return jv;
}


template<size_t DOF>
void Wam<DOF>::gravityCompensate(bool compensate) {
	bt_wam_setgcomp(wam, compensate);
}

template<size_t DOF>
void Wam<DOF>::moveHome() {
	bt_wam_movehome(wam);
}

template<size_t DOF>
bool Wam<DOF>::moveIsDone() {
	return bt_wam_moveisdone(wam);
}

template<size_t DOF>
void Wam<DOF>::idle() {
	bt_wam_idle(wam);
}


template<size_t DOF>
std::map<struct bt_wam_local*, Wam<DOF>*> Wam<DOF>::activeWams;

template<size_t DOF>
int Wam<DOF>::handleCallback(struct bt_wam_local* wamLocal) {
	activeWams[wamLocal]->readSensors();
	return 0;
}


template<size_t DOF>
void Wam<DOF>::readSensors() {
	jp_type jp;
	jv_type jv;

	// TODO(dc): make conversion to/from GSL vectors
	for (size_t i = 0; i < DOF; ++i) {
		jp[i] = gsl_vector_get(wamLocal->jposition, i);
		jv[i] = gsl_vector_get(wamLocal->jvelocity, i);
	}

	this->jpOutputValue->setValue(jp);
	this->jvOutputValue->setValue(jv);
//	std::cout << jp << std::endl;
}

template<size_t DOF>
void Wam<DOF>::operate()
{
	++operateCount;

	if (this->input.valueDefined()) {
		const jt_type& jt = this->input.getValue();
		for (size_t i = 0; i< DOF; ++i) {
			gsl_vector_set(wamLocal->jtorque, i,
					gsl_vector_get(wamLocal->jtorque, i) + jt[i]);
		}
	}
}


}
