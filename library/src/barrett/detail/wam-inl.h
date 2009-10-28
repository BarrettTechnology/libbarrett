/*
 * wam-inl.h
 *
 *  Created on: Sep 25, 2009
 *      Author: dc
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
#include "../systems/abstract/single_io.h"


namespace barrett {

// TODO(dc): some of theses members should be inline

template<size_t DOF>
Wam<DOF>::Wam() :
		systems::SingleIO<jt_type, ja_type>(),
//		systems::SingleIO<units::JointTorques<DOF>, units::JointAngles<DOF> >(),
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
void Wam<DOF>::gravityCompensate(bool compensate) {
	bt_wam_setgcomp(wam, compensate);
}

template<size_t DOF>
void Wam<DOF>::moveHome() {
	bt_wam_movehome(wam);
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
	ja_type ja;
	for (size_t i = 0; i < DOF; ++i) {
		ja[i] = gsl_vector_get(wamLocal->jposition, i);
	}
	this->outputValue->setValue(ja);
}

template<size_t DOF>
void Wam<DOF>::operate()
{
	++operateCount;

	const jt_type& jt = this->input.getValue();
	for (size_t i = 0; i< DOF; ++i) {
		gsl_vector_set(wamLocal->jtorque, i,
				gsl_vector_get(wamLocal->jtorque, i) + jt[i]);
	}
}


}
