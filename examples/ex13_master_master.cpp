/*
 * master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
//#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>

#include "ex13_master_master.h"


using namespace barrett;
using detail::waitForEnter;


void ghcEntryPoint(GimbalsHandController* ghc, const char* otherIp);

bool validate_args(int argc, char** argv) {
	if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <otherip> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}

	return true;
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	const jp_type SYNC_POS(0.0);  // the position each WAM should move to before linking


	MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
	systems::connect(wam.jpOutput, mm.input);


	wam.gravityCompensate();


	std::vector<std::string> autoCmds;
	if (argc == 3) {  // auto init
//		if (pm.foundGimbalsHandController()) {
		if (true) {
			autoCmds.push_back("g");
		}
		if (pm.foundHand()) {
			autoCmds.push_back("h");
		}
//		autoCmds.push_back("l");
	}

	boost::thread* ghcThread = NULL;
	std::string line;
	v_type gainTmp;
	while (true) {
		if (autoCmds.empty()) {
			printf(">>> ");
			std::getline(std::cin, line);
		} else {
			line = autoCmds.back();
			autoCmds.pop_back();
		}

		switch (line[0]) {
		case 'l':
			if (mm.isLinked()) {
				mm.unlink();
			} else {
				wam.moveTo(SYNC_POS);

//				printf("Press [Enter] to start sending joint positions.");
//				waitForEnter();

				printf("Press [Enter] to link with the other WAM.");
				waitForEnter();
				mm.tryLink();
				wam.trackReferenceSignal(mm.output);

				usleep(100000);  // wait an execution cycle or two
				if (mm.isLinked()) {
					printf("Linked with remote WAM.\n");
				} else {
					printf("WARNING: Linking was unsuccessful.\n");
				}
			}

			break;

		case 'g':
			if (ghcThread == NULL) {
//				if (pm.foundGimbalsHandController()) {
				if (true) {
//					ghcThread = new boost::thread(ghcEntryPoint, pm.getGimbalsHandController(), argv[1]);
					ghcThread = new boost::thread(ghcEntryPoint, (GimbalsHandController*)NULL, argv[1]);
					printf("Started Gimbals Hand Controller thread.\n");
				} else {
					printf("WARNING: No Gimbals Hand Controller found.\n");
				}
			} else {
				ghcThread->interrupt();
				ghcThread->join();
				delete ghcThread;
				ghcThread = NULL;
				printf("Stopped Gimbals Hand Controller thread.\n");
			}

			break;

		case 't':
			size_t jointIndex;
			{
				size_t jointNumber;
				std::cout << "\tJoint: ";
				std::cin >> jointNumber;
				jointIndex = jointNumber - 1;

				if (jointIndex >= DOF) {
					std::cout << "\tBad joint number: " << jointNumber;
					break;
				}
			}

			char gainId;
			std::cout << "\tGain identifier (p, i, or d): ";
			std::cin >> line;
			gainId = line[0];

			std::cout << "\tCurrent value: ";
			switch (gainId) {
			case 'p':
				gainTmp = wam.jpController.getKp();
				break;
			case 'i':
				gainTmp = wam.jpController.getKi();
				break;
			case 'd':
				gainTmp = wam.jpController.getKd();
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}
			std::cout << gainTmp[jointIndex] << std::endl;

			std::cout << "\tNew value: ";
			std::cin >> gainTmp[jointIndex];
			switch (gainId) {
			case 'p':
				wam.jpController.setKp(gainTmp);
				break;
			case 'i':
				wam.jpController.setKi(gainTmp);
				break;
			case 'd':
				wam.jpController.setKd(gainTmp);
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}

			break;
			
		default:
			printf("\n");
			printf("    'l' to toggle linking with other WAM\n");
			printf("    'h' to toggle Hand thread\n");
			printf("    'g' to toggle Gimbals Hand Controller thread\n");
			printf("    't' to tune control gains\n");

			break;
		}
	}


	delete ghcThread;

	return 0;
}


void ghcEntryPoint(GimbalsHandController* ghc, const char* otherIp) {
	while ( !boost::this_thread::interruption_requested() ) {
/*
		printf("%d,%d  %d,%d  %d,%d  %d,%d  %f\n",
				ghc.getThumbOpen(), ghc.getThumbClose(),
				ghc.getPointerOpen(), ghc.getPointerClose(),
				ghc.getMiddleOpen(), ghc.getMiddleClose(),
				ghc.getRockerUp(), ghc.getRockerDown(),
				ghc.getKnob());

*/
		usleep(100000);
	}
}
