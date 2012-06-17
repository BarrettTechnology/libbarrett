/*
 * inverse_dynamics_test_teach_and_play_accel.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: dc
 */

#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()

#include <boost/tuple/tuple.hpp>

#include <libconfig.h++>

#include <barrett/log.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>


using namespace barrett;
using systems::connect;


bool validate_args(int argc, char** argv) {
	if (argc != 2) {
		printf("Usage: %s <fileName>\n", argv[0]);
		return false;
	}
	return true;
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}


	libconfig::Config config;
	config.readFile("inverse_dynamics_test.conf");

//	wam.jpController.setControlSignalLimit(jt_type(0.0));  // Turn off saturation

//	systems::PIDController<jp_type, ja_type> pid(config.lookup("control_joint"));
//	systems::InverseDynamics<DOF> id(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["dynamics"]);
//	connect(wam.jpOutput, pid.feedbackInput);
//	connect(wam.jvOutput, id.jvInput);
//	connect(wam.kinematicsBase.kinOutput, id.kinInput);
//	connect(pid.controlOutput, id.input);
//	wam.supervisoryController.registerConversion(systems::makeIOConversion(pid.referenceInput, id.output));


	wam.gravityCompensate();
	usleep(250000);
	pm.getSafetyModule()->setVelocityLimit(1.5);


	// Build spline between recorded points
	typedef boost::tuple<double, jp_type> jp_sample_type;
	systems::Ramp moveTime(pm.getExecutionManager(), 1.0);
	log::Reader<jp_sample_type> inputLr("/home/robot/libbarrett/sandbox/fastMove.bin");
	std::vector<jp_sample_type> vec;
	for (size_t i = 0; i < inputLr.numRecords(); ++i) {
		vec.push_back(inputLr.getRecord());
	}
	math::Spline<jp_type> spline(vec);

	// First, move to the starting position
	wam.moveTo(spline.eval(spline.initialS()));

	// Then play back the recorded motion
	moveTime.setOutput(spline.initialS());

	systems::Callback<double, jp_type> trajectory(boost::ref(spline));
	connect(moveTime.output, trajectory.input);
	wam.trackReferenceSignal(trajectory.output);


	double omega_p = 180.0;
	systems::FirstOrderFilter<jp_type> hp1;
	hp1.setHighPass(jp_type(omega_p), jp_type(omega_p));
	systems::FirstOrderFilter<jp_type> hp2;
	hp2.setHighPass(jp_type(omega_p), jp_type(omega_p));
	systems::Gain<jp_type, double, ja_type> changeUnits(1.0);
	const LowLevelWam<DOF>& llw = wam.getLowLevelWam();
	systems::Gain<ja_type, sqm_type, jt_type> driveInertias(llw.getJointToMotorPositionTransform().transpose() * v_type(config.lookup("drive_inertias")).asDiagonal() * llw.getJointToMotorPositionTransform());
	systems::InverseDynamics<DOF> id(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["dynamics"]);
	systems::Summer<jt_type> idSum;

	connect(trajectory.output, hp1.input);
	connect(hp1.output, hp2.input);
	connect(hp2.output, changeUnits.input);
	pm.getExecutionManager()->startManaging(hp2);

	connect(changeUnits.output, id.input);
	connect(changeUnits.output, driveInertias.input);
	connect(wam.jvOutput, id.jvInput);
	connect(wam.kinematicsBase.kinOutput, id.kinInput);
	connect(id.output, idSum.getInput(0));
	connect(driveInertias.output, idSum.getInput(1));

	wam.jpController.getKp() *= 1.0;
	wam.jpController.getKi() *= 1.0;
	wam.jpController.getKd() *= 1.0;


	systems::FirstOrderFilter<jv_type> hp3;
	omega_p = 100.0;
	wam.jvFilter.setLowPass(jv_type(omega_p));
	hp3.setHighPass(jp_type(omega_p), jp_type(omega_p));
	pm.getExecutionManager()->startManaging(hp3);
	systems::Gain<jv_type, double, ja_type> changeUnits2(1.0);
	systems::PIDController<ja_type, jt_type> jaController(config.lookup("joint_acceleration_control"));

	connect(wam.jvOutput, hp3.input);
	connect(hp3.output, changeUnits2.input);
	connect(changeUnits2.output, jaController.feedbackInput);
	wam.supervisoryController.registerConversion(&jaController);

//	wam.trackReferenceSignal(reference.output);
	sleep(1);
	connect(idSum.output, wam.input);
	wam.trackReferenceSignal(changeUnits.output);
	sleep(1);

//	wam.jpController.getKp()[1] = 0.0;
//	wam.jpController.getKi()[1] = 0.0;
//	wam.jpController.getKd()[1] = 0.0;


	systems::Ramp time(pm.getExecutionManager(), 1.0);

	systems::TupleGrouper<double, jp_type, jp_type, jt_type> tg;
	connect(time.output, tg.template getInput<0>());
	connect(trajectory.output, tg.template getInput<1>());
	connect(wam.jpOutput, tg.template getInput<2>());
//	connect(changeUnits.output, tg.template getInput<1>());
//	connect(changeUnits2.output, tg.template getInput<2>());
//	connect(wam.jtSum.output, tg.template getInput<3>());
	connect(jaController.controlOutput, tg.template getInput<3>());
//	connect(hp2.output, tg.template getInput<2>());
//	connect(id.output, tg.template getInput<3>());

	typedef boost::tuple<double, jp_type, jp_type, jt_type> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	time.start();
	connect(tg.output, logger.input);
	printf("Logging started.\n");


	usleep(100000);
//	reference.setValue(endPos);
//	wam.moveTo(startPos, v_type(0.0), endPos, false, 2.667, 0.2);
	moveTime.start();

	while (trajectory.input.getValue() < spline.finalS()) {
		usleep(100000);
	}

	usleep(500000);


	logger.closeLog();
	printf("Logging stopped.\n");

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);


	wam.moveHome();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
