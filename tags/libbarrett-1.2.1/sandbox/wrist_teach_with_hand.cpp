/*
 * wrist_teach_with_hand.cpp
 *
 *  Created on: Dec 29, 2011
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;


enum TP_STATE {
	WAIT_FOR_TEACH,
	TEACHING,
	WAIT_FOR_PLAY,
	PLAYING,
	DONE_PLAYING,
	IDLE
} tpState = WAIT_FOR_TEACH;
bool loop = false;


bool handPause = true;

void displayEntryPoint() {
	printf("\n");
	printf("Commands:\n");
	printf("  t    Start teaching a new trajectory\n");
	printf("  p    Play back recorded trajectory\n");
	printf("  l    Loop recorded trajectory\n");
	printf("  s    Stop teaching, stop playing\n");
	printf("  At any time, press [Enter] to toggle pause/play for the Hand.\n");
	printf("\n");

	std::string line;
	while (true) {
		printf(">> ");
		std::getline(std::cin, line);

		if (line.size() == 0) {
			// Toggle Hand motion
			handPause = !handPause;
		} else {
			switch (line[0]) {
			case 't':
				tpState = TEACHING;
				break;
			case 'l':
				loop = true;
				//break;
			case 'p':
				tpState = PLAYING;
				break;
			case 's':
				loop = false;
				if (tpState == TEACHING) {
					tpState = WAIT_FOR_PLAY;
				} else if (tpState == PLAYING  ||  tpState == DONE_PLAYING) {
					tpState = IDLE;
				}
				break;
			default:
				break;
			}
		}
	}
}


void handEntryPoint(Hand& hand) {
	std::vector<Hand::jp_type> positions;

	const double O = 0.0;
	const double C = 2.4;
	const double SC = M_PI;

	positions.push_back(Hand::jp_type(O, O, O, O));
	positions.push_back(Hand::jp_type(C, C, C, O));
	positions.push_back(Hand::jp_type(O, O, O, O));
	positions.push_back(Hand::jp_type(O, O, O, SC));
	positions.push_back(Hand::jp_type(C, C, C, SC));
	positions.push_back(Hand::jp_type(C*0.3, C*0.7, C*0.5, SC));
	positions.push_back(Hand::jp_type(C*0.7, C*0.3, C*0.5, SC));

	size_t i = 0;
	while(true) {
		i = (i + 1) % positions.size();
		sleep(3);
		while (handPause) {
			usleep(250000);
		}
		hand.trapezoidalMove(positions[i]);
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	typedef boost::tuple<double, jp_type> jp_sample_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	const double T_s = pm.getExecutionManager()->getPeriod();


	if (pm.foundHand()) {
		printf("Press [Enter] to initialize the Hand. (Make sure it has room!)");
		waitForEnter();
		pm.getHand()->initialize();

		boost::thread handThread(handEntryPoint, boost::ref(*pm.getHand()));
	} else {
		printf("WARNING: No Hand found!\n");
	}
	boost::thread displayThread(displayEntryPoint);

	while (true) {
		systems::Ramp time(pm.getExecutionManager());
		systems::TupleGrouper<double, jp_type> jpLogTg;
		systems::PeriodicDataLogger<jp_sample_type> jpLogger(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 10*T_s), 10);

		while (tpState != TEACHING) {
			usleep(100000);
		}

		{
			// Make sure the Systems are connected on the same execution cycle
			// that the time is started. Otherwise we might record a bunch of
			// samples all having t=0; this is bad because the Spline requires time
			// to be monotonic.
			BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

			connect(time.output, jpLogTg.template getInput<0>());
			connect(wam.jpOutput, jpLogTg.template getInput<1>());
			connect(jpLogTg.output, jpLogger.input);
			time.start();
		}
		printf("--> Recording\n");


		while (tpState != WAIT_FOR_PLAY) {
			usleep(100000);
		}
		jpLogger.closeLog();
		disconnect(jpLogger.input);
		printf("--> Done recording\n");


		// Build spline between recorded points
		log::Reader<jp_sample_type> lr(tmpFile);
		std::vector<jp_sample_type> vec;
		for (size_t i = 0; i < lr.numRecords(); ++i) {
			vec.push_back(lr.getRecord());
		}
		math::Spline<jp_type> spline(vec);

		systems::Callback<double, jp_type> trajectory(boost::ref(spline));
		connect(time.output, trajectory.input);


		while (true) {
			while (tpState != PLAYING) {
				usleep(100000);
			}
			printf("--> Moving to start position\n");

			// First, move to the starting position
			wam.moveTo(spline.eval(spline.initialS()));

			// Then play back the recorded motion
			time.stop();
			time.setOutput(spline.initialS());

			wam.trackReferenceSignal(trajectory.output);

			time.start();
			printf("--> Playing\n");

//			while (trajectory.input.getValue() < spline.finalS()) {
//				usleep(100000);
//			}


			while (tpState == PLAYING) {
				usleep(100000);

				if (trajectory.input.getValue() >= spline.finalS()) {
					tpState = DONE_PLAYING;
				}
			}
			printf("--> Done playing\n");
			if (tpState == DONE_PLAYING  &&  loop) {
				tpState = PLAYING;
				printf("--> Looping!\n");
				continue;
			}
			while (tpState == DONE_PLAYING) {
				usleep(100000);
			}
			if (tpState == PLAYING) {
				continue;
			} else if (tpState == IDLE) {
				wam.idle();
				printf("--> Idle\n");

				while (tpState == IDLE) {
					usleep(100000);
				}
				if (tpState == PLAYING) {
					continue;
				} else {
					break;
				}
			} else {
				wam.idle();
				break;
			}
		}
	}

	std::remove(tmpFile);
	return 0;
}


const size_t DOF = 3;
const int FIRST_WRIST_ID = 5;
const char CONFIG_FILE[] = "wam3.conf";
const char CONFIG_PATH[] = "wam3";


int main(int argc, char** argv) {
	BARRETT_UNITS_TYPEDEFS(DOF);

	installExceptionHandler();


	ProductManager pm(CONFIG_FILE);
	pm.wakeAllPucks();

	std::vector<Puck*> wamPucks;
	for (int i = FIRST_WRIST_ID; i < FIRST_WRIST_ID + (int)DOF; ++i) {
		wamPucks.push_back(pm.getPuck(i));
		if (wamPucks.back() == NULL) {
			printf(">>> ERROR: No Wrist found. Is the power on and the CAN cable connected?\n");
			return 1;
		}
	}

	// Take the other WAM Pucks off of the BGRP_WAM group so they won't respond
	// to position requests. This allows us to test this code while the Wrist is
	// mounted on a WAM. Power cycle the Pucks to return to the normal grouping.
	for (int i = ProductManager::FIRST_WAM_ID; i < FIRST_WRIST_ID; ++i) {
		Puck* puck = pm.getPuck(i);
		if (puck != NULL) {
			puck->setProperty(Puck::GRPC, 0);
		}
	}

	printf(">>> The Wrist needs to be zeroed. Please move it to its home position, then press [Enter].");
	waitForEnter();
	std::vector<int> torqueGroupIds(1, PuckGroup::BGRP_UPPER_WAM);
	systems::Wam<DOF> wam(pm.getExecutionManager(), wamPucks, NULL, pm.getConfig().lookup(CONFIG_PATH), torqueGroupIds);
	pm.startExecutionManager();

	printf(">>> Press [Enter] to Activate the Wrist. WARNING: This bypasses the Safety Module!");
	waitForEnter();
	wam.getLowLevelWam().getPuckGroup().setProperty(Puck::MODE, MotorPuck::MODE_TORQUE);


	int ret = wam_main(argc, argv, pm, wam);


	wam.getLowLevelWam().getPuckGroup().setProperty(Puck::MODE, MotorPuck::MODE_IDLE);
	return ret;
}
