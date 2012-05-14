/*
 * autohome.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: CJ Valle
 */

#include <iostream>
#include <string>
#include <cmath>
#include <cstdio>

#include <native/timer.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#define BARRETT_SMF_DONT_PROMPT_ON_ZEROING // prevents the "move to home position" prompt
#include <barrett/standard_main_function.h>

using namespace barrett;

const double SPEED = 0.5;  // radians per second

template<size_t DOF>
class Autohome {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
private:
	jp_type setPoint, min, max, jointRange;
	ProductManager& pm;
	systems::Wam<DOF>& wam;

	// Wraps usleep to guarantee it returns on time, and to provide SECONDS as an input
	void sleep(double duration) {
		unsigned long total = duration*1000000;
		unsigned long passed = 0;
		unsigned long remaining = total;
		RTIME start = rt_timer_read();
		while (true) {
			usleep(remaining);
			passed = (rt_timer_read() - start) / 1000;
			if (passed >= total) break;
			remaining = total - passed;
		}
	}

	// Watches the given joint in the WAM for a stop and returns the final (stopped) position
	double monitor(int joint) {
		jp_type last = wam.getJointPositions();
		sleep(1);
		while(fabs(last[joint]-wam.getJointPositions()[joint]) > .0001)
		{
			last = wam.getJointPositions();
			sleep(.5);
		}
		return last[joint];
	}

	// Moves the given joint to the given position, handling joint stops smoothly
	void moveTheWAM(int joint, double position, bool blocking, bool onJoint = false) {
		jp_type other;
		for(size_t i=0; i<DOF; i++) other[i] = setPoint[i];
		if (onJoint)
		{
			if(setPoint[joint] > 0) other[joint] = wam.getJointPositions()[joint]+0.2;
			else other[joint] = wam.getJointPositions()[joint]-0.2;
		}
		setPoint[joint] = position;
		if (joint==6) wam.moveTo(other, /*jv_type(0.0),*/ setPoint, blocking, 2*SPEED, 3*SPEED);
		else wam.moveTo(other, /*jv_type(0.0),*/ setPoint, blocking, SPEED, SPEED);
	}

	// Figures out the zero position for the given joint
	void goToZero(int joint) {
		moveTheWAM(joint, 10.0, false);
		max[joint] = monitor(joint);
		moveTheWAM(joint, -10.0, false, true);
		min[joint] = monitor(joint);
		moveTheWAM(joint, getZeroPosition(joint), true, true);
	}

	double getZeroPosition(int joint) {
		if (joint == 4) return max[joint]-1.35;
		if (joint == 3) return min[joint]+.9;
		return (max[joint]+min[joint])/2;
	}

public:
	// Constructor for Autohome object
	Autohome(ProductManager& p, systems::Wam<DOF>& w) :
		pm(p), wam(w) {

		// Assume the WAM is not properly zeroed.
		pm.getSafetyModule()->setWamZeroed(false);

		setPoint = wam.getJointPositions();

		// Set control signal limits in the WAM.  These values prevent erratic behavior.
		wam.jpController.getControlSignalLimit()[1] = 50;
		if (DOF >= 7) {
			wam.jpController.getControlSignalLimit()[6] = 3;
		}

		// Set up expected joint ranges
		jointRange[0] = 5.2;
		jointRange[1] = 4.0;
		jointRange[2] = 5.6;
		jointRange[3] = 4.0;
		if (DOF > (size_t)4) {
			jointRange[4] = 6.0;
			jointRange[5] = 3.2;
			jointRange[6] = 6.0;
		}
	}

	//  Brings the robot to home, returns int representing any failures by joint, with each bit representing one joint
	int home() {
		// Open joint 4 all the way to make sure it doesn't collide
		moveTheWAM(3, -10.0, false);
		double temp = monitor(3);
		moveTheWAM(3, temp+0.9, true, true);

       	// If there's a hand, move it into a "fist" orientation
		if (pm.foundHand()) {
			Hand* hand = pm.getHand();
			hand->initialize();
			hand->close(Hand::SPREAD);
			hand->close(Hand::GRASP, false);
		}

		// Handle the wrist if it's present
		if (DOF > 4)
		{
			// Do joint 7
			goToZero(6);
			// Do joint 5
			goToZero(4);
			// Do joint 6
			goToZero(5);
		}

		// Do joint 2
		goToZero(1);
		// Do joint 4
		goToZero(3);

		//Do joint 3
		goToZero(2);
		//Do joint 1
		goToZero(0);

		// THE ROBOT IS NOW AT ZERO

		// Check to be sure we zeroed successfully
		int err = 0;
		for(size_t i=0; i<DOF; i++) {
			if (fabs(max[i]-min[i]) < jointRange[i]) {
				err |= 1 << i;
			}
		}

		// Now move to the home position
		if (pm.foundHand()) {
			pm.getHand()->trapezoidalMove(Hand::jp_type(M_PI/2.0), Hand::GRASP, false);
		}

		jp_type home = setPoint + wam.getHomePosition();
		wam.moveTo(setPoint, /*jv_type(0.0),*/ home, true, 0.5, 0.5);

		// Pause at the end for idle
		pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

		return err;
	}
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	Autohome<DOF> autohome(pm, wam);
	int err = autohome.home();
	std::cout << "Returned error code " << err << "\n";
	return err;
}
