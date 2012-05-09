/*
 * log_ft_data.cpp
 *
 *  Created on: Jul 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <syslog.h>
#include <signal.h>
#include <unistd.h>

#include <native/task.h>
#include <native/timer.h>

#include <boost/thread.hpp>
#include <boost/ref.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stacktrace.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

typedef boost::tuple<double, cf_type, ct_type> tuple_type;

bool g_Going = true;  // Global


void stopThreads(int sig) {
	g_Going = false;
}

void warnOnSwitchToSecondaryMode(int)
{
	syslog(LOG_ERR, "WARNING: Switched out of RealTime. Stack-trace:");
	detail::syslog_stacktrace();
	std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

void ftThreadEntryPoint(bool* going, double T_s, ForceTorqueSensor& fts, log::RealTimeWriter<tuple_type>* lw, int windowSize, int* numSamples, tuple_type* sum) {
	tuple_type t;

	rt_task_shadow(new RT_TASK, NULL, 10, 0);
	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);

	RTIME now = rt_timer_read();
	RTIME lastUpdate = now;

	while (*going) {
		rt_task_wait_period(NULL);
		now = rt_timer_read();
		fts.update(true);  // Do a realtime update (no sleeping while waiting for messages)

		boost::get<0>(t) = ((double) now - lastUpdate) * 1e-9;
		boost::get<1>(t) = fts.getForce();
		boost::get<2>(t) = fts.getTorque();

		if (lw != NULL) {
			lw->putRecord(t);
		} else {
			if (*numSamples == 0) {
				boost::get<1>(*sum).setZero();
				boost::get<2>(*sum).setZero();
			}
			if (*numSamples < windowSize) {
				boost::get<1>(*sum) += boost::get<1>(t);
				boost::get<2>(*sum) += boost::get<2>(t);
				++(*numSamples);
			}
		}

		lastUpdate = now;
	}

	rt_task_set_mode(T_WARNSW, 0, NULL);
}

void showUsageAndExit(const char* programName) {
	printf("Usage: %s {-f <fileName> | -a <windowSize>} [<samplePeriodInSeconds>]\n", programName);
	printf("    -f <fileName>    Log data to a file\n");
	printf("    -a <windowSize>  Print statistics on segments of data\n");
	exit(0);
}


int main(int argc, char** argv) {
	char* outFile = NULL;
	double T_s = 0.002;  // Default: 500Hz
	bool fileMode = false;
	int windowSize = 0;

	if (argc == 4) {
		T_s = std::atof(argv[3]);
	} else if (argc != 3) {
		showUsageAndExit(argv[0]);
	}

	printf("Sample period: %fs\n", T_s);
	if (strcmp(argv[1], "-f") == 0) {
		fileMode = true;
		outFile = argv[2];
		printf("Output file: %s\n", outFile);
	} else if (strcmp(argv[1], "-a") == 0) {
		fileMode = false;
		windowSize = atoi(argv[2]);
		printf("Window size: %d\n", windowSize);
	} else {
		showUsageAndExit(argv[0]);
	}
	printf("\n");


	signal(SIGXCPU, &warnOnSwitchToSecondaryMode);

	char tmpFile[] = "/tmp/btXXXXXX";
	log::RealTimeWriter<tuple_type>* lw = NULL;
	if (fileMode) {
		signal(SIGINT, &stopThreads);

		if (mkstemp(tmpFile) == -1) {
			printf("ERROR: Couldn't create temporary file!\n");
			return 1;
		}
		lw = new barrett::log::RealTimeWriter<tuple_type>(tmpFile, T_s);
	}
	int numSamples = windowSize, numSets = 0;
	tuple_type sum;

	ProductManager pm;
	if ( !pm.foundForceTorqueSensor() ) {
		printf("ERROR: No Force-Torque Sensor found!\n");
		return 1;
	}
	boost::thread ftThread(ftThreadEntryPoint, &g_Going, T_s, boost::ref(*pm.getForceTorqueSensor()), lw, windowSize, &numSamples, &sum);

	if (fileMode) {
		printf(">>> Logging data. Press [Ctrl-C] to exit.\n");
	} else {
		printf(">>> Press [Enter] to start a new sample. Press [Ctrl-C] to exit.\n\n");
		printf("ID,FX,FY,FZ,TX,TY,TZ");

		while (g_Going) {
			waitForEnter();
			numSamples = 0;
			while (numSamples != windowSize) {
				usleep(100000);
			}

			boost::get<1>(sum) /= windowSize;
			boost::get<2>(sum) /= windowSize;

			printf("%d,%f,%f,%f,%f,%f,%f", ++numSets, boost::get<1>(sum)[0], boost::get<1>(sum)[1], boost::get<1>(sum)[2], boost::get<2>(sum)[0], boost::get<2>(sum)[1], boost::get<2>(sum)[2]);
		}
	}

	ftThread.join();
	printf("\n");

	if (fileMode) {
		delete lw;

		log::Reader<tuple_type> lr(tmpFile);
		lr.exportCSV(outFile);
		printf("Output written to %s.\n", outFile);

		std::remove(tmpFile);
	}

	return 0;
}
