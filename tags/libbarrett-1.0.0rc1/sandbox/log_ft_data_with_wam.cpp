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

#include <barrett/cdlbt/bus.h>
#include <barrett/cdlbt/bus_can.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/detail/stacktrace.h>


using namespace barrett;

const int HAND_MOVE_TIME = 1;  // seconds
const size_t DOF = 4;
const double T_s = 0.002;
const int ID = BT_BUS_PUCK_ID_FT;
const RTIME FLIGHT_TIME = 75000;
const int FW_VERS = 148;  // bogus value until the F/T firmware implements VERS
const int NUM_SENSORS = 6;  // FX, FY, FZ, TX, TY, TZ
const int NUM_MESSAGES = 2;  // F/T readings are packed into 2 messages
struct bt_bus_can* dev = NULL;

bool going = true;

typedef boost::tuple<double, math::Vector<NUM_SENSORS>::type> tuple_type;
barrett::log::RealTimeWriter<tuple_type>* lw;



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


void ftThread();
void handControl();


int main() {
	const char outFile[] = "ft.csv";

	libconfig::Config config;
	config.readFile("/etc/barrett/wam4.conf");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	systems::Wam<DOF> wam(config.lookup("wam"));
	dev = wam.wam.wambot->bus->dev;


	// start the main loop!
	rtem.start();


	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
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


int twoByte2int(unsigned char lsb, unsigned char msb) {
	int res = ((int)msb << 8)  |  lsb;

	if (res & 0x00008000) {
		res |= 0xffff0000;
	}

	return res;
}

void ftThread() {
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

	bt_bus_can_set_property(dev, ID, plist->FT, 0);  // tarre (do this whenever necessary)


	RTIME t_0 = rt_timer_read();
	RTIME t_1 = t_0;

	while (going) {
		// strain gauges
		bt_bus_can_async_get_property(dev, ID, plist->FT);

		t_0 = rt_timer_read();
		boost::get<0>(t) = ((double) t_0 - t_1) * 1e-9;
		t_1 = t_0;

		for (int i = 0; i < NUM_MESSAGES; ++i) {
			bt_bus_can_async_read(dev, &id, NULL, &value, NULL, data, 1, 0);

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

		lw->putRecord(t);
	}

	rt_task_set_mode(T_WARNSW, 0, NULL);
	bt_bus_properties_destroy(plist);
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

