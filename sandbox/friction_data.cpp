/*
 * friction_data.cpp
 *
 *  Created on: Jun 4, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>


using namespace barrett;
using systems::connect;

const size_t DOF = 4;
const double T_s = 0.002;
const char BIN_FILE[] = "/tmp/friction.bin";
const char CSV_FILE[] = "/tmp/friction.csv";

const int JIDX = 3;  // joint 4
const double VEL = 0.5;


BARRETT_UNITS_TYPEDEFS(DOF);
typedef math::Vector<3>::type log_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	libconfig::Config config;
	config.readFile("/etc/barrett/wam4.conf");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;

	// instantiate Systems
	systems::Wam<DOF> wam(config.lookup("wam"));
	systems::ArraySplitter<jp_type> jpS;
	systems::ArraySplitter<jv_type> jvS;
	systems::ArraySplitter<jt_type> jtS;
	systems::Constant<log_type> ltConst(log_type(-1e5));
	systems::ArrayEditor<log_type> ae;
	systems::PeriodicDataLogger<log_type> dl(
			new log::RealTimeWriter<log_type>(BIN_FILE, T_s),
			1);

	// connect Systems
	connect(wam.jpOutput, jpS.input);
	connect(jpS.getOutput(JIDX), ae.getElementInput(0));
	connect(wam.jvOutput, jvS.input);
	connect(jvS.getOutput(JIDX), ae.getElementInput(1));
	connect(wam.jtSum.output, jtS.input);
	connect(jtS.getOutput(JIDX), ae.getElementInput(2));

	connect(ltConst.output, ae.input);
	connect(ae.output, dl.input);

	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << "Press [Enter] to move to start position.\n";
	waitForEnter();
	jp_type jp;
//	jp << 0, -M_PI/2.0, -M_PI/2.0, 3, 0,0,0;
	jp << 0, -M_PI/2.0, -M_PI/2.0, 3;
	wam.moveTo(jp);


	// use position controller
	jp[JIDX] = -0.8;
	wam.moveTo(jp, true, VEL, 0.5);
	jp[JIDX] = 3;
	wam.moveTo(jp, true, VEL, 0.5);


	// use velocity controller
//	math::Vector<DOF>::type gains;
//	gains = wam.jpController.getKp();
//	gains[JIDX] = 0;
//	wam.jpController.setKp(gains);
//	gains = wam.jpController.getKi();
//	gains[JIDX] = 0;
//	wam.jpController.setKi(gains);
//	gains = wam.jpController.getKd();
//	gains[JIDX] = 0;
//	wam.jpController.setKd(gains);
//
//	gains.setConstant(0.0);
//	gains[JIDX] = wam.jvController1.getKp()[JIDX];
//	wam.jvController1.setKp(gains);
//	gains[JIDX] = wam.jvController1.getKi()[JIDX];
//	wam.jvController1.setKi(gains);
//	gains[JIDX] = wam.jvController1.getKd()[JIDX];
//	wam.jvController1.setKd(gains);
//
//	systems::ExposedOutput<jv_type> jvCommand(jv_type(-VEL));
//	connect(jvCommand.output, wam.jvController1.referenceInput);
//	connect(wam.jvController2.output, wam.input);
//
//	std::cout << "Press [Enter] to reverse.\n";
//	waitForEnter();
//	jvCommand.setValue(jv_type(VEL));


	std::cout << "Press [Enter] to exit.\n";
	waitForEnter();

	rtem.stop();
	dl.closeLog();

	log::Reader<log_type> lr(BIN_FILE);
	lr.exportCSV(CSV_FILE);

	return 0;
}
