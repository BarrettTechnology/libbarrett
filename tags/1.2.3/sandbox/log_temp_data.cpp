/*
 * log_temp_data.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: dc
 */

#include <cstdio>
#include <vector>

#include <signal.h>
#include <native/timer.h>

#include <boost/thread.hpp>
#include <boost/array.hpp>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>


using namespace barrett;


bool validate_args(int argc, char** argv) {
	if (argc != 2) {
		printf("Usage: %s <fileName>\n", argv[0]);
		printf("  fileName:  File to record temperature data to\n");
		return false;
	} else {
		return true;
	}
}

void logEntryPoint(const PuckGroup& pg, const char* outFile) {
	const size_t MAX_PG_SIZE = ProductManager::MAX_WAM_DOF;
	typedef boost::tuple<double, boost::array<int, MAX_PG_SIZE>, boost::array<int, MAX_PG_SIZE> > tuple_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		exit(1);
	}

	// Become a shadowed Xenomai task so we can use rt_timer services.
	pg.getPucks()[0]->getProperty(Puck::STAT);
	RTIME start = rt_timer_read();

	tuple_type data;
	log::Writer<tuple_type> logWriter(tmpFile);
	while ( !boost::this_thread::interruption_requested() ) {
		boost::get<0>(data) = (rt_timer_read() - start) * 1e-9;
		pg.getProperty(Puck::TEMP, boost::get<1>(data).data());
		pg.getProperty(Puck::THERM, boost::get<2>(data).data());
		logWriter.putRecord(data);

		usleep(5000000);
	}
	logWriter.close();

	log::Reader<tuple_type> logReader(tmpFile);
	logReader.exportCSV(outFile);
	printf("Output written to %s.\n", outFile);
	std::remove(tmpFile);
}

bool going = true;
void sigint(int) {
	going = false;
	printf("\nStopping...\n");
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// Disable torque saturation
//	wam.jpController.setControlSignalLimit(v_type(0.0));


	jp_type tmp;
	std::vector<jp_type> poses;

	// J1
	tmp.setZero();
	tmp[0] = -2.6;
	tmp[1] = -M_PI_2;
	tmp[3] = M_PI_2;
	poses.push_back(tmp);
	tmp[0] = -tmp[0];
	poses.push_back(tmp);

	// J2
	tmp.setZero();
	tmp[1] = -1.9;
	poses.push_back(tmp);
	tmp[1] = -tmp[1];
	poses.push_back(tmp);

	// J3
	tmp.setZero();
	tmp[1] = M_PI_2;
	tmp[2] = -2.5;
	tmp[3] = M_PI_2;
	poses.push_back(tmp);
	tmp[2] = -tmp[2];
	poses.push_back(tmp);

	// J4 forward curls
	tmp.setZero();
	tmp[1] = -M_PI_2;
	tmp[3] = -0.9;
	poses.push_back(tmp);
	tmp[3] = 3.1;
	if (DOF >= 6) {
		tmp[5] = -0.7;
	}
	poses.push_back(tmp);

	// J4 backward curls
	tmp.setZero();
	tmp[1] = M_PI_2;
	tmp[3] = -0.9;
	poses.push_back(tmp);
	tmp[3] = M_PI_2;
	poses.push_back(tmp);


	boost::thread logThread(logEntryPoint, boost::ref(wam.getLowLevelWam().getPuckGroup()), argv[1]);
	signal(SIGINT, sigint);
	printf("Recording!\n");

	const size_t NUM_REPS = 30;
	printf("Total reps: %zu\n", NUM_REPS);
	printf("REP    PROGRESS\n");

	size_t i;
	jp_type a, b;
	jp_type prev = wam.getJointPositions();
	for (size_t reps = 0; going  &&  reps < NUM_REPS; ++reps) {
		printf("%3zu    ", reps + 1);
		fflush(stdout);

		i = 0;
		while (going  &&  i < poses.size()) {
			a = poses[i++];
			b = poses[i++];
			for (size_t j = 0; going  &&  j < 4; ++j) {
				printf(".");
				fflush(stdout);

				wam.moveTo(prev, /*jv_type(),*/ a, true, 0.5, 0.5);
				prev = a;

				if ( !going ) {
					break;
				}

				wam.moveTo(prev, /*jv_type(),*/ b, true, 0.5, 0.5);
				prev = b;
			}
			wam.moveTo(prev, /*jv_type(),*/ wam.getHomePosition(), true, 0.5, 0.5);
			prev = wam.getHomePosition();
		}

		printf("\n");
	}

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	logThread.interrupt();
	printf("Recording stopped.\n");
	logThread.join();

	return 0;
}
