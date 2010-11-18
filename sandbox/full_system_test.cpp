/*
 * full_system_test.cpp
 *
 *  Created on: Nov 18, 2010
 *      Author: dc
 */

#include <vector>

#include <unistd.h>

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
void moveWam(const bool& going, BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	while (going) {
		sleep(1);
	}
}

void moveHand(const bool& going, Hand& hand) {
	Hand::jp_type jp0, jp1;
	jp0.setZero();
	jp1 << 30000, 30000, 30000, 10000;

	while (going) {
		sleep(3);
		hand.trapezoidalMove(jp1, false);
		sleep(3);
		hand.trapezoidalMove(jp0, false);

//		hand.updatePosition();
//		hand.updateStrain();
//		std::cout << hand.getPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";
	}

	hand.idle();
}

void readHand(const bool& going, Hand& hand) {
	while (going) {
		usleep(10000);
		hand.updatePosition();
		hand.updateStrain();
	}
}

void readTact(const bool& going, Hand& hand) {
	while (going) {
		usleep(10000);
		hand.updateTactFull();
	}
}

void readFTS(const bool& going, ForceTorqueSensor& fts) {
	while (going) {
		usleep(10000);
		fts.update();
	}
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	BusManager bm;
	bool going = true;
	std::vector<boost::thread*> threads;


	// WAM
	bm.waitForWam();
	if (bm.foundWam4()) {
		printf("Starting WAM4...\n");
		threads.push_back(new boost::thread(moveWam<4>, ref(going), ref(bm), ref(*bm.getWam4(false))));
	} else if (bm.foundWam7()) {
		printf("Starting WAM7...\n");
		threads.push_back(new boost::thread(moveWam<7>, ref(going), ref(bm), ref(*bm.getWam7(false))));
	} else {
		printf(">>> WARNING: No WAM found\n");
	}

	// Hand
	if (bm.foundHand()) {
		printf("Starting Hand...\n");
		Hand& hand = *bm.getHand();

		printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)\n");
		waitForEnter();
		hand.initialize();

		threads.push_back(new boost::thread(moveHand, ref(going), ref(hand)));
		threads.push_back(new boost::thread(readHand, ref(going), ref(hand)));
		threads.push_back(new boost::thread(readTact, ref(going), ref(hand)));
	} else {
		printf(">>> WARNING: No Hand found\n");
	}

	// F/T
	if (bm.foundForceTorqueSensor()) {
		printf("Starting F/T Sensor...\n");
		threads.push_back(new boost::thread(readFTS, ref(going), ref(*bm.getForceTorqueSensor())));
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
