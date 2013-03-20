/*
 * point_to_point_moves.cpp
 *
 *  Created on: Feb 9, 2012
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cassert>

#include <boost/thread.hpp>
#include <boost/ref.hpp>

#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


void printMenu() {
	printf("Commands:\n");
	printf("  p  Play/pause\n");
	printf("  h  Move home\n");
	printf("  i  Idle (release position constraints)\n");
	printf("  q  Quit\n");
}


bool play = false;
template<size_t DOF>
void moveEntryPoint(systems::Wam<DOF>& wam, const libconfig::Setting& setting) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	assert(setting.isList()  ||  setting.isArray());

	std::vector<jp_type, Eigen::aligned_allocator<jp_type> > jpVec;
	for (int i = 0; i < setting.getLength(); ++i) {
		jpVec.push_back(jp_type(setting[i]));
	}

	bool resume = true;
	size_t i = 0;
	assert(jpVec.size() >= 1);
	while ( !boost::this_thread::interruption_requested() ) {
		if (play  &&  wam.moveIsDone()) {
			if ( !resume ) {
				btsleep(1.0);
				i = (i + 1) % jpVec.size();
			}
			wam.moveTo(jpVec[i], false);
			resume = false;
		} else if ( !play  &&  !wam.moveIsDone() ) {
			wam.moveTo(wam.getJointPositions());
			resume = true;
		}

		btsleep(0.1);
	}
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	libconfig::Config config;
	config.readFile("point_to_point_moves.conf");
	boost::thread moveThread(moveEntryPoint<DOF>, boost::ref(wam), boost::ref(config.lookup(pm.getWamDefaultConfigPath())));


	wam.gravityCompensate();
	printMenu();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'p':
			play = !play;
			if (play) {
				printf("Play!\n");
			} else {
				printf("Pause!\n");
			}
			break;

		case 'h':
			play = false;
			while ( !wam.moveIsDone() ) {
				btsleep(0.1);
			}
			wam.moveHome();
			break;

		case 'i':
			printf("WAM idled.\n");
			play = false;
			wam.idle();
			break;

		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			break;

		default:
			if (line.size() != 0) {
				printf("Unrecognized option.\n");
				printMenu();
			} else {
				std::cout << wam.getJointPositions() << "\n";
				Eigen::Quaterniond to(wam.getToolOrientation());
				printf("%+7.4f %+7.4fi %+7.4fj %+7.4fk", to.w(), to.x(), to.y(), to.z());
			}
			break;
		}
	}

	moveThread.interrupt();
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
