/*
 * full_system_test.cpp
 *
 *  Created on: Nov 18, 2010
 *      Author: dc
 */

#include <vector>

#include <unistd.h>
#include <native/timer.h>

#include <curses.h>

#include <boost/thread.hpp>
#include <boost/ref.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
using boost::ref;
const bool USE_CURESES = false;


class RatePrinter {
public:
	RatePrinter(int _y, int _x) :
		y(_y), x(_x) {
		rps.push_back(this);

		now = rt_timer_read();
		lastUpdate = now;
		runningTotal = 0;
		avgTenSec = 0;
		iterations = 0;
	}
	// TODO: dtor that removes this from rps.

	void update() {
		now = rt_timer_read();
		runningTotal += now - lastUpdate;
		rate = 1e9 / ((double) now - lastUpdate);

		iterations++;
		if (runningTotal >= 1e9) {
			avgTenSec = (double) iterations / runningTotal * 1e9;
			runningTotal = 0;
			iterations = 0;
		}

		lastUpdate = now;
	}

	void print() {
		if (USE_CURESES) {
			mvprintw(y,x, "%10.2f  %10.2lf", rate, avgTenSec);
		} else {
//			printf("%10.2f,%10.2lf, ", rate, avgTenSec);
			printf("%7.2lf,", avgTenSec);
		}
	}

	static void printAll() {
		for (size_t i = 0; i < rps.size(); ++i) {
			rps[i]->print();
		}
	}

protected:
	static std::vector<RatePrinter*> rps;

	int y;
	int x;

	RTIME now;
	RTIME lastUpdate;
	RTIME runningTotal;
	double rate, avgTenSec;
	int iterations;
};

std::vector<RatePrinter*> RatePrinter::rps;


void runDisplay()
{
	if (USE_CURESES) {
		initscr();
		curs_set(0);
		noecho();
		timeout(0);

		mvprintw(0,0, "WAM");
		mvprintw(1,0, "F/T Sensor");
		mvprintw(2,0, "Hand");
	}

	while ( !boost::this_thread::interruption_requested() ) {
		RatePrinter::printAll();

		if (USE_CURESES) {
			refresh();
			usleep(200000);
		} else {
			printf("\n");
			sleep(1);
		}
	}

	if (USE_CURESES) {
		endwin();
	}
}

template<size_t DOF>
void readWam(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	RatePrinter rp(0,20);

	jp_type jp;
	jv_type jv;
	jt_type jt;
	cp_type tp;
	Eigen::Quaterniond to;

	while ( !boost::this_thread::interruption_requested() ) {
		jp = wam.getJointPositions();
		jv = wam.getJointVelocities();
		jt = wam.getJointTorques();
		tp = wam.getToolPosition();
		to = wam.getToolOrientation();

		rp.update();
		usleep(100000);
	}
}

void readFTS(ForceTorqueSensor& fts) {
	RatePrinter rp(1,20);

	while ( !boost::this_thread::interruption_requested() ) {
		fts.update();

		rp.update();
		usleep(100);
	}
}

void readHand(Hand& hand) {
	RatePrinter rp(2,20);

//	int puckTemp[Hand::DOF];
//	int motorTemp[Hand::DOF];

	while ( !boost::this_thread::interruption_requested() ) {
		hand.update();
//		hand.getPuckGroup().getProperty(Puck::TEMP, puckTemp);
//		hand.getPuckGroup().getProperty(Puck::THERM, motorTemp);

		rp.update();
		usleep(100);
	}
}

void moveHand(Hand& hand) {
	Hand::jp_type tmp;
	std::vector<Hand::jp_type> pos;

	tmp.setZero();
	pos.push_back(tmp);

	tmp.setConstant(M_PI);
	tmp[3] = 0;
	pos.push_back(tmp);

	tmp.setZero();
	pos.push_back(tmp);

	tmp.setZero();
	tmp[3] = M_PI;
	pos.push_back(tmp);

	tmp.setConstant(M_PI);
	pos.push_back(tmp);

	tmp.setZero();
	tmp[3] = M_PI;
	pos.push_back(tmp);


	while ( !boost::this_thread::interruption_requested() ) {
		for (size_t i = 0; i < pos.size()  &&  !boost::this_thread::interruption_requested(); ++i) {
			sleep(3);
			hand.trapezoidalMove(pos[i], false);
		}
	}
}

template<size_t DOF>
void moveAndRead(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	std::vector<boost::thread*> threads;
	Hand& hand = *pm.getHand();

	wam.gravityCompensate();
	jp_type jp(0.0);
	jp[1] = -M_PI_2;
	jp[3] = M_PI_2;
	wam.moveTo(jp);


	// WAM
	threads.push_back(new boost::thread(readWam<DOF>, ref(pm), ref(wam)));

	// F/T
	if (pm.foundForceTorqueSensor()) {
		printf("Starting F/T Sensor...\n");
		threads.push_back(new boost::thread(readFTS, ref(*pm.getForceTorqueSensor())));
	} else {
		printf(">>> WARNING: No F/T Sensor found\n");
	}

	// Hand
	if (pm.foundHand()) {
		printf("Starting Hand...\n");
		hand.initialize();
		threads.push_back(new boost::thread(moveHand, ref(hand)));
		threads.push_back(new boost::thread(readHand, ref(*pm.getHand())));
	} else {
		printf(">>> WARNING: No Hand found\n");
	}

	threads.push_back(new boost::thread(runDisplay));


	sleep(30);


	// clean up
	for (size_t i = 0; i < threads.size(); ++i) {
		threads[i]->interrupt();
		threads[i]->join();
	}

	if (pm.foundHand()) {
		hand.open();
		hand.close(Hand::SPREAD);
//		hand.trapezoidalMove(Hand::jp_type(M_PI/2.0), Hand::GRASP);
	}
	wam.moveHome();
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm;
	pm.wakeAllPucks();


	if (pm.foundWam4()) {
		moveAndRead(pm, *pm.getWam4());
	} else if (pm.foundWam7()) {
		moveAndRead(pm, *pm.getWam7());
	} else {
		printf(">>> WARNING: No WAM found\n");
	}


	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
}
