/*
 * full_system_test.cpp
 *
 *  Created on: Nov 18, 2010
 *      Author: dc
 */

#include <vector>

#include <boost/thread.hpp>
#include <boost/ref.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/force_torque_sensor.h>


using namespace barrett;
using detail::waitForEnter;
using boost::ref;


template<size_t DOF>
void runWam(const bool& going, BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	while (going) {
		sleep(1);
	}
}

void runHand(const bool& going, Hand& hand) {
	while (going) {
		sleep(1);
	}
}

void runFTS(const bool& going, ForceTorqueSensor& fts) {
	while (going) {
		sleep(1);
	}
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	BusManager bm;
	bool going = true;
	std::vector<boost::thread*> threads;


	// WAM
//	bm.waitForWam();
	if (bm.foundWam4()) {
		threads.push_back(new boost::thread(runWam<4>, ref(going), ref(bm), ref(*bm.getWam4())));
		printf(">>> Starting WAM4...\n");
	} else if (bm.foundWam7()) {
		threads.push_back(new boost::thread(runWam<7>, ref(going), ref(bm), ref(*bm.getWam7())));
		printf(">>> Starting WAM7...\n");
	} else {
		printf(">>> WARNING: No WAM found\n");
	}

	// Hand
	if (bm.foundHand()) {
		threads.push_back(new boost::thread(runHand, ref(going), ref(*bm.getHand())));
		printf(">>> Starting Hand...\n");
	} else {
		printf(">>> WARNING: No Hand found\n");
	}

	// F/T
	if (bm.foundForceTorqueSensor()) {
		threads.push_back(new boost::thread(runFTS, ref(going), ref(*bm.getForceTorqueSensor())));
		printf(">>> Starting F/T Sensor...\n");
	} else {
		printf(">>> WARNING: No F/T Sensor found\n");
	}


	printf(">>> Press [Enter] to stop.\n");
	waitForEnter();


	// clean up
	going = false;
	for (size_t i = 0; i < threads.size(); ++i) {
		threads[i]->join();
	}
}
