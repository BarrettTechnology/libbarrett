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
#include <barrett/wambot/wambot_phys.h>

#include "../detail/debug.h"
#include "../units.h"
#include "../systems/abstract/system.h"


namespace barrett {

// TODO(dc): some of these members should be inline

template<size_t DOF>
Wam<DOF>::Wam() :
	System(),
	input(this), jpOutput(this, &jpOutputValue),
	jvOutput(this, &jvOutputValue), operateCount(), jtSink(this), wam(NULL), wamLocal(NULL)
{
	// initialize syslog
	openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

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
	return jp_type(wamLocal->jposition);
}

template<size_t DOF>
units::JointVelocities<DOF> Wam<DOF>::getJointVelocities()
{
	return jv_type(wamLocal->jvelocity);
}


template<size_t DOF>
void Wam<DOF>::gravityCompensate(bool compensate)
{
	bt_wam_setgcomp(wam, compensate);
}

template<size_t DOF>
void Wam<DOF>::moveHome()
{
	bt_wam_movehome(wam);
}

template<size_t DOF>
bool Wam<DOF>::moveIsDone()
{
	return bt_wam_moveisdone(wam);
}

template<size_t DOF>
void Wam<DOF>::idle()
{
	bt_wam_idle(wam);
}


template<size_t DOF>
std::map<struct bt_wam_local*, Wam<DOF>*> Wam<DOF>::activeWams;

bool uglyFlag = false;

template<size_t DOF>
int Wam<DOF>::handleCallback(struct bt_wam_local* wamLocal)
{
	activeWams[wamLocal]->readSensors();
	uglyFlag = true;
	activeWams[wamLocal]->operate();
	return 0;
}


template<size_t DOF>
void Wam<DOF>::readSensors()
{
	int err;
	double time = 1e-9 * bt_os_rt_get_time();

    /* Grab the current joint positions */
    bt_wambot_update( wamLocal->wambot );

    /* Evaluate kinematics
     * NOTE: Should this be common for all refgens/controllers?
     *       It's definitely needed for Barrett Dynamics,
     *       but this is encapsulated in Controllers right now ... */
    bt_kinematics_eval(wamLocal->kin, wamLocal->wambot->jposition, wamLocal->wambot->jvelocity);

    /* Get the position from the current controller */
    bt_control_get_position(wamLocal->con_active);

    /* Zero the torque
     * (this is here in case a refgen wants to tweak it,
     * which really isn't what we should be doing ... */
    gsl_vector_set_zero( wamLocal->wambot->jtorque );

    /* If there's an active trajectory, grab the reference into the joint controller
     * Note: this is a while loop for the case where the refgen is done,
     *       and we move on to the next refgen. */
    while (wamLocal->refgen_active && !wamLocal->teaching)
    {
       err = bt_refgen_eval( wamLocal->refgen_active,
                             time - wamLocal->refgen_start_time,
                             wamLocal->con_active->reference );

       if (!err) break;

       if (err == 1) /* finished */
       {
          if ( (wamLocal->refgen_active == wamLocal->refgen_tempmove)
              && (wamLocal->refgen_loaded) )
          {
             wamLocal->refgen_start_time = time;
             bt_refgen_start(wamLocal->refgen_loaded);
             wamLocal->refgen_active = wamLocal->refgen_loaded;
          }
          else
          {
             wamLocal->refgen_active = 0;
          }
       }
    }

    /* Do the active controller */
    bt_control_eval( wamLocal->con_active, wamLocal->wambot->jtorque, time );

    /* Do gravity compensation (if flagged) */
    if (wamLocal->gcomp) bt_calgrav_eval( wamLocal->grav, wamLocal->wambot->jtorque );

	this->jpOutputValue->setValue(jp_type(wamLocal->jposition));
	this->jvOutputValue->setValue(jv_type(wamLocal->jvelocity));
}

template<size_t DOF>
bool Wam<DOF>::inputsValid()
{
	return true;
}

template<size_t DOF>
void Wam<DOF>::operate()
{
//	++operateCount;
	readSensors();
//	if (!uglyFlag) {
//		return;
//	}
//	uglyFlag = false;
//	++operateCount;
//
//	if (this->input.valueDefined()) {
//		const jt_type& jt = this->input.getValue();
//		for (size_t i = 0; i< DOF; ++i) {
//			gsl_vector_set(wamLocal->jtorque, i,
//					gsl_vector_get(wamLocal->jtorque, i) + jt[i]);
//		}
//	}
//
//
//
//    /* Apply the current joint torques */
//    bt_wambot_setjtor( wamLocal->wambot );
}

template<size_t DOF>
void Wam<DOF>::invalidateOutputs()
{
	/* do nothing */
}


}
