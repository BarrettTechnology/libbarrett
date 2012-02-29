/*
 * log_sg_data.cpp
 *
 *  Created on: Apr 26, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include <native/task.h>
#include <native/timer.h>

#include <barrett/products/product_manager.h>
#include <barrett/log.h>
#include <barrett/detail/stl_utils.h>


using namespace barrett;
using detail::waitForEnter;


RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}

void canThread();

double T_s = 0.002;
const int NUM_SENSORS = 9;  // 6 strain gauges, 3 thermistors

bool going = true;
bool fileMode = false;
int windowSize, numSamples, numSets = 0;
math::Vector<NUM_SENSORS>::type sum;
struct bt_bus_can* dev = NULL;

typedef boost::tuple<double, math::Vector<NUM_SENSORS>::type> tuple_type;
barrett::log::RealTimeWriter<tuple_type>* lw;
ForceTorqueSensor* fts = NULL;


int main(int argc, char** argv) {
	char* outFile = NULL;

	if (argc == 4) {
		T_s = std::atof(argv[3]);
	} else if (argc != 3) {
		printf("Usage: %s {-f <fileName> | -a <windowSize>} [<samplePeriodInSeconds>]\n", argv[0]);
		printf("    -f <fileName>    Log data to a file\n");
		printf("    -a <windowSize>  Print statistics on segments of data\n");
		return 0;
	}

	printf("Sample period: %fs\n", T_s);
	if (strcmp(argv[1], "-f") == 0) {
		fileMode = true;
		outFile = argv[2];
		printf("Output file: %s\n", outFile);
	} else if (strcmp(argv[1], "-a") == 0) {
		fileMode = false;
		windowSize = atoi(argv[2]);
		numSamples = windowSize;
		printf("Window size: %d\n", windowSize);
	}
	printf("\n");


	ProductManager pm;
	if ( !pm.foundForceTorqueSensor() ) {
		printf("ERROR: No Force-Torque Sensor found!\n");
		return 1;
	}
	fts = pm.getForceTorqueSensor();

	if (fileMode) {
		printf(">>> Press [Enter] to start collecting data.\n");
		waitForEnter();

		char tmpFile[] = "/tmp/btXXXXXX";
		if (mkstemp(tmpFile) == -1) {
			printf("ERROR: Couldn't create temporary file!\n");
			return 1;
		}
		lw = new barrett::log::RealTimeWriter<tuple_type>(tmpFile, T_s);
		boost::thread t(canThread);


		printf(">>> Press [Enter] to stop collecting data.\n");
		waitForEnter();

		going = false;
		t.join();
		delete lw;

		barrett::log::Reader<tuple_type> lr(tmpFile);
		lr.exportCSV(outFile);

		std::remove(tmpFile);
	} else {
		boost::thread t(canThread);

		printf(">>> Press [Enter] to start a sample.\n");
		printf("ID,SG1,SG2,SG3,SG4,SG5,SG6,T1,T2,T3\n");

		while (true) {
			waitForEnter();
			numSamples = 0;
			while (numSamples != windowSize) {
				usleep(100000);
			}

			sum /= windowSize;
			printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f", ++numSets, sum[0], sum[1], sum[2], sum[3], sum[4], sum[5], sum[6], sum[7], sum[8]);
		}
	}

	return 0;
}


void canThread() {
	tuple_type t;

	int id = fts->getPuck()->getId();
	const bus::CommunicationsBus& bus = fts->getPuck()->getBus();

	rt_task_shadow(new RT_TASK, NULL, 10, 0);


	// collect data
	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s));

	RTIME t_0 = rt_timer_read();
	RTIME t_1 = t_0;

	while (going) {
		rt_task_wait_period(NULL);

		t_0 = rt_timer_read();
		boost::get<0>(t) = ((double) t_0 - t_1) * 1e-9;
		t_1 = t_0;


		// strain gauges
		boost::get<1>(t)[0] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG1, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[1] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG2, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[2] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG3, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[3] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG4, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[4] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG5, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[5] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG6, Puck::PT_ForceTorque, 0), true);

		// temp sensors
		boost::get<1>(t)[6] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::T1, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[7] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::T2, Puck::PT_ForceTorque, 0), true);
		boost::get<1>(t)[8] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::T3, Puck::PT_ForceTorque, 0), true);


		if (fileMode) {
			lw->putRecord(t);
		} else {
			if (numSamples == 0) {
				sum.setConstant(0.0);
			}
			if (numSamples < windowSize) {
				sum += boost::get<1>(t);
				++numSamples;
			}
		}
	}

	rt_task_set_mode(T_WARNSW, 0, NULL);
}
