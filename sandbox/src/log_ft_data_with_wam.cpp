/*
 * log_ft_data_with_wam.cpp
 *
 *  Created on: Jun 28, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <native/task.h>
#include <native/timer.h>
#include <libconfig.h++>

#include <barrett/bus/bus.h>
#include <barrett/bus/bus_can.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/wam.h>
#include <barrett/detail/stacktrace.h>


using namespace barrett;

const int HAND_MOVE_TIME = 1;  // seconds
const size_t DOF = 4;
const double T_s = 0.002;
const int ID = BT_BUS_PUCK_ID_FT;
const RTIME FLIGHT_TIME = 75000;
const int NUM_SENSORS = 6;  // 6 strain gauges, 3 thermistors
struct bt_bus_can* dev = NULL;

bool going = true;
math::Vector<NUM_SENSORS>::type sum;

typedef boost::tuple<double, math::Vector<NUM_SENSORS>::type> tuple_type;
barrett::log::RealTimeWriter<tuple_type>* lw;



void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

void warnOnSwitchToSecondaryMode(int)
{
	syslog(LOG_ERR, "WARNING: Switched out of RealTime. Stack-trace:");
	syslog_stacktrace();
	std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}


void ftThread();
void handControl();


int main() {
	const char outFile[] = "ft.csv";

	libconfig::Config config;
	config.readFile("/etc/wam/wam4-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));
	dev = wam.wam.wambot->bus->dev;


	// start the main loop!
	rtem.start();


	char tmpFile[L_tmpnam];
	if (std::tmpnam(tmpFile) == NULL) {
		printf("Couldn't create temporary file!\n");
		return 1;
	}
	lw = new barrett::log::RealTimeWriter<tuple_type>(tmpFile, T_s);

	boost::thread ftT(ftThread);
	boost::thread hcT(handControl);


	printf("Press [Enter] to compensate for gravity.");
	waitForEnter();
	wam.gravityCompensate();

	printf("Press [Enter] to exit.");
	waitForEnter();


	going = false;
	ftT.join();
	hcT.join();
	delete lw;

	barrett::log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(outFile);

	std::remove(tmpFile);


	rtem.stop();

	return 0;
}


void ftThread() {
	int id, property;
	long value;
	tuple_type t;

	rt_task_shadow(new RT_TASK, NULL, 10, 0);
//	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
//	rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s));

	RTIME t_0 = rt_timer_read();
	RTIME t_1 = t_0;

	while (going) {
		// strain gruages
		bt_bus_can_async_get_property(dev, ID, 34);
		rt_timer_spin(FLIGHT_TIME);
		bt_bus_can_async_get_property(dev, ID, 35);
		rt_timer_spin(FLIGHT_TIME);
		bt_bus_can_async_get_property(dev, ID, 36);
		rt_timer_spin(FLIGHT_TIME);
		bt_bus_can_async_get_property(dev, ID, 37);
		rt_timer_spin(FLIGHT_TIME);
		bt_bus_can_async_get_property(dev, ID, 38);
		rt_timer_spin(FLIGHT_TIME);
		bt_bus_can_async_get_property(dev, ID, 39);

		// temp sensors
//		rt_timer_spin(FLIGHT_TIME);
//		bt_bus_can_async_get_property(dev, ID, 40);
//		rt_timer_spin(FLIGHT_TIME);
//		bt_bus_can_async_get_property(dev, ID, 41);
//		rt_timer_spin(FLIGHT_TIME);
//		bt_bus_can_async_get_property(dev, ID, 9);

//		rt_task_wait_period(NULL);

		t_0 = rt_timer_read();
		boost::get<0>(t) = ((double) t_0 - t_1) * 1e-9;
		t_1 = t_0;

		for (int i = 0; i < NUM_SENSORS; ++i) {
			bt_bus_can_async_read(dev, &id, &property, &value, NULL, 1, 0);

			if (property == 9) {
				boost::get<1>(t)[8] = value;
			} else {
				boost::get<1>(t)[property-34] = value;
			}
		}

		lw->putRecord(t);
	}

	rt_task_set_mode(T_WARNSW, 0, NULL);
}

void handControl() {
	int speed = 100;
	int spreadspeed = 20;


	rt_task_shadow(new RT_TASK, NULL, 9, 0);

	printf("Waking hand pucks ...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	usleep((long)1e6);

	printf("Setting TSTOP ...\n");
	bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 78, 50); // Set TSTOP to 50 ms
	usleep((long)1e3);


	while (going) {
		bt_bus_can_set_property(dev, 14, 44, spreadspeed);
		bt_bus_can_set_property(dev, 14, 8, 4);
		sleep(HAND_MOVE_TIME);
		bt_bus_can_set_property(dev, 14, 44, -spreadspeed);
		bt_bus_can_set_property(dev, 14, 8, 4);
		sleep(HAND_MOVE_TIME);
		bt_bus_can_set_property(dev, 14, 44, 0);
		bt_bus_can_set_property(dev, 14, 8, 4);

		sleep(HAND_MOVE_TIME);

		for (int i = 11; i <= 13; ++i) {
			bt_bus_can_set_property(dev, i, 44, -speed);
			bt_bus_can_set_property(dev, i, 8, 4);
		}
		sleep(HAND_MOVE_TIME);
		for (int i = 11; i <= 13; ++i) {
			bt_bus_can_set_property(dev, i, 44, speed);
			bt_bus_can_set_property(dev, i, 8, 4);
		}
		sleep(HAND_MOVE_TIME);
		for (int i = 11; i <= 13; ++i) {
			bt_bus_can_set_property(dev, i, 44, 0);
			bt_bus_can_set_property(dev, i, 8, 4);
		}

		sleep(HAND_MOVE_TIME);
	}
}

