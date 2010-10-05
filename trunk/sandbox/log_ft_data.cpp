/*
 * log_ft_data.cpp
 *
 *  Created on: Jul 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include <native/task.h>
#include <native/timer.h>

#include <barrett/cdlbt/bus.h>
#include <barrett/cdlbt/bus_can.h>
#include <barrett/log.h>
#include <barrett/detail/stacktrace.h>

using namespace barrett;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

void warnOnSwitchToSecondaryMode(int)
{
	syslog(LOG_ERR, "WARNING: Switched out of RealTime. Stack-trace:");
	detail::syslog_stacktrace();
	std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}

void canThread();

double T_s = 0.002;
const int ID = BT_BUS_PUCK_ID_FT;
const int CAN_PORT = 1;
const RTIME FLIGHT_TIME = 75000;
const int FW_VERS = 148;  // bogus value until the F/T firmware implements VERS
const int NUM_SENSORS = 6;  // 6 strain gauges, 3 thermistors
const int NUM_MESSAGES = 2;  // 2 from strain gauges, 3 from the thermistors

bool going = true;
bool fileMode = false;
int windowSize, numSamples, numSets = 0;
math::Vector<NUM_SENSORS>::type sum;
struct bt_bus_can* dev = NULL;

typedef boost::tuple<double, math::Vector<NUM_SENSORS>::type> tuple_type;
barrett::log::RealTimeWriter<tuple_type>* lw;


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


	mlockall(MCL_CURRENT|MCL_FUTURE);
	signal(SIGXCPU, &warnOnSwitchToSecondaryMode);

	if (bt_bus_can_create(&dev, CAN_PORT)) {
		printf("Couldn't open the CAN bus on port %d.\n", CAN_PORT);
		return -1;
	}

	if (fileMode) {
		printf(">>> Press [Enter] to start collecting data.\n");
		waitForEnter();

		char tmpFile[L_tmpnam];
		if (std::tmpnam(tmpFile) == NULL) {
			printf("Couldn't create temporary file!\n");
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
//		printf("ID,FX,FY,FZ,TX,TY,TZ,T1,T2,T3\n");
		printf("ID,FX,FY,FZ,TX,TY,TZ\n");

		while (true) {
			waitForEnter();
			numSamples = 0;
			while (numSamples != windowSize) {
				usleep(100000);
			}

			sum /= windowSize;
//			printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f", ++numSets, sum[0], sum[1], sum[2], sum[3], sum[4], sum[5], sum[6], sum[7], sum[8]);
			printf("%d,%f,%f,%f,%f,%f,%f", ++numSets, sum[0], sum[1], sum[2], sum[3], sum[4], sum[5]);
		}
	}

	return 0;
}

int twoByte2int(unsigned char lsb, unsigned char msb) {
	int res = ((int)msb << 8)  |  lsb;

	if (res & 0x00008000) {
		res |= 0xffff0000;
	}

	return res;
}

void canThread() {
	struct bt_bus_properties* plist;
	int id;
	long value;
	unsigned char data[8];
	tuple_type t;

	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	// init sensor
	bt_bus_can_set_property(dev, ID, 5, 2);
	usleep(1000000);

	if (bt_bus_properties_create(&plist, FW_VERS)) {
		printf("Couldn't create property list.\n");
		exit(-1);
	}

	bt_bus_can_set_property(dev, ID, plist->FT, 0);  // tarre


	// collect data
	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s));

	RTIME t_0 = rt_timer_read();
	RTIME t_1 = t_0;

	while (going) {
		// strain gauges
		bt_bus_can_async_get_property(dev, ID, plist->FT);

//		// temp sensors
//		rt_timer_spin(FLIGHT_TIME);
//		bt_bus_can_async_get_property(dev, ID, plist->T1);
//		rt_timer_spin(FLIGHT_TIME);
//		bt_bus_can_async_get_property(dev, ID, plist->T2);
//		rt_timer_spin(FLIGHT_TIME);
//		bt_bus_can_async_get_property(dev, ID, plist->T3);

		rt_task_wait_period(NULL);

		t_0 = rt_timer_read();
		boost::get<0>(t) = ((double) t_0 - t_1) * 1e-9;
		t_1 = t_0;

		for (int i = 0; i < NUM_MESSAGES; ++i) {
			bt_bus_can_async_read(dev, &id, NULL, &value, NULL, data, 1, 1);

			if ((id & 0x041F) == 0x040a) {
				boost::get<1>(t)[0] = twoByte2int(data[0], data[1]);
				boost::get<1>(t)[1] = twoByte2int(data[2], data[3]);
				boost::get<1>(t)[2] = twoByte2int(data[4], data[5]);
			} else if ((id & 0x041F) == 0x040b) {
				boost::get<1>(t)[3] = twoByte2int(data[0], data[1]);
				boost::get<1>(t)[4] = twoByte2int(data[2], data[3]);
				boost::get<1>(t)[5] = twoByte2int(data[4], data[5]);
			} else {
				printf("Unexpected CAN message!\n");
			}
		}

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

	bt_bus_properties_destroy(plist);
}
