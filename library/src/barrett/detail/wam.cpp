/*
 * wam.cpp
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
#include "../wam.h"


namespace barrett {

// TODO(dc): some of theses members should be inline

Wam::Wam() :
	systems::SingleIO<units::JointTorques, units::JointAngles>(),
	operateCount(), wam(NULL), wamLocal(NULL)
{
	// initialize syslog
	openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

	// lock memory
	mlockall(MCL_CURRENT | MCL_FUTURE);

	// open the WAM
	units::DOF = 7;  // TODO(dc): BAD BAD BAD! :(
	bt_wam_create(&wam, "wamg");
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

Wam::~Wam()
{
	// stop receiving callbacks, then unregister the Wam
	bt_wam_local_set_callback(wamLocal, NULL);
	activeWams.erase(wamLocal);

	bt_wam_destroy(wam);
}

void Wam::gravityCompensate(bool compensate) {
	bt_wam_setgcomp(wam, compensate);
}

void Wam::moveHome() {
	bt_wam_movehome(wam);
}

void Wam::idle() {
	bt_wam_idle(wam);
}


std::map<struct bt_wam_local*, Wam*> Wam::activeWams;

int Wam::handleCallback(struct bt_wam_local* wamLocal) {
	activeWams[wamLocal]->readSensors();
	return 0;
}


void Wam::readSensors() {
	units::JointAngles ja;
	for (size_t i = 0; i < ja.size(); ++i) {
		ja[i] = gsl_vector_get(wamLocal->jposition, i);
	}
	this->outputValue->setValue(ja);
}

void Wam::operate()
{
	++operateCount;

	const units::JointTorques& jt = input.getValue();
	for (size_t i = 0; i< jt.size(); ++i) {
		gsl_vector_set(wamLocal->jtorque, i,
				gsl_vector_get(wamLocal->jtorque, i) + jt[i]);
	}
}


}
