/*
 * log_hand_jp.cpp
 *
 *  Created on: Mar 30, 2011
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>
#include <native/timer.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


bool validate_args(int argc, char** argv) {
	if (argc != 2) {
		printf("Usage: %s <fileName>\n", argv[0]);
		printf("  fileName:  File to record joint position data to\n");
		return false;
	} else {
		return true;
	}
}

void logEntryPoint(Hand& hand, const char* outFile) {
	typedef boost::tuple<double, Hand::jp_type, Hand::jp_type, boost::tuple<double, double, double, double, double, double, double, double> > tuple_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		exit(1);
	}

	// Become a shadowed Xenomai task so we can use rt_timer services.
	hand.update();
	RTIME start = rt_timer_read();

	tuple_type data;
	log::Writer<tuple_type> logWriter(tmpFile);
	while ( !boost::this_thread::interruption_requested() ) {
		boost::get<0>(data) = (rt_timer_read() - start) * 1e-9;

		hand.update();

		boost::get<1>(data) = hand.getInnerLinkPosition();
		boost::get<2>(data) = hand.getOuterLinkPosition();

		boost::get<0>(boost::get<3>(data)) = hand.getPrimaryEncoderPosition()[0];
		boost::get<1>(boost::get<3>(data)) = hand.getPrimaryEncoderPosition()[1];
		boost::get<2>(boost::get<3>(data)) = hand.getPrimaryEncoderPosition()[2];
		boost::get<3>(boost::get<3>(data)) = hand.getPrimaryEncoderPosition()[3];
		boost::get<4>(boost::get<3>(data)) = hand.getSecondaryEncoderPosition()[0];
		boost::get<5>(boost::get<3>(data)) = hand.getSecondaryEncoderPosition()[1];
		boost::get<6>(boost::get<3>(data)) = hand.getSecondaryEncoderPosition()[2];
		boost::get<7>(boost::get<3>(data)) = hand.getSecondaryEncoderPosition()[3];

		logWriter.putRecord(data);
		usleep(1000);
	}
	logWriter.close();

	log::Reader<tuple_type> logReader(tmpFile);
	logReader.exportCSV(outFile);
	printf("Output written to %s.\n", outFile);
	std::remove(tmpFile);
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	Hand::jp_type currentPos(0.0);
	Hand::jp_type nextPos(M_PI);
	nextPos[3] = 0;


	if (!pm.foundHand()) {
		printf("ERROR: No Hand found!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();

	wam.gravityCompensate();

	printf("Press [Enter] to initialize Hand. (Make sure it has room!)");
	waitForEnter();
	hand.initialize();


	boost::thread logThread(logEntryPoint, boost::ref(hand), argv[1]);
	printf("Recording!\n");
	printf("Press [Enter] to open/close Hand, \"x [Enter]\" to exit.\n");

	std::string line;
	while (true) {
		std::getline(std::cin, line);
		if (line.size() == 0) {
			hand.trapezoidalMove(nextPos, false);
			std::swap(currentPos, nextPos);
		} else {
			break;
		}
	}

	printf("Recording stopped.\n");
	logThread.interrupt();
	logThread.join();


	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
