/*
 * inverse_dynamics_test.cpp
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

	const jp_type startPos(config.lookup("start_pos"));
	const jp_type endPos(config.lookup("end_pos"));


	systems::PIDController<jp_type, ja_type> pid(config.lookup("control_joint"));
	systems::InverseDynamics<DOF> id(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["dynamics"]);
	connect(wam.jpOutput, pid.feedbackInput);
	connect(wam.jvOutput, id.jvInput);
	connect(wam.kinematicsBase.kinOutput, id.kinInput);
	connect(pid.controlOutput, id.input);
	wam.supervisoryController.registerConversion(systems::makeIOConversion(pid.referenceInput, id.output));


	wam.gravityCompensate();
	usleep(250000);
	wam.moveTo(startPos);

	systems::ExposedOutput<jp_type> reference(startPos);
	wam.trackReferenceSignal(reference.output);
	sleep(2);
//	wam.jpController.getKp()[1] = 0.0;
//	wam.jpController.getKi()[1] = 0.0;
//	wam.jpController.getKd()[1] = 0.0;


	systems::Ramp time(pm.getExecutionManager(), 1.0);

	systems::TupleGrouper<double, jp_type, jp_type, jt_type> tg;
	connect(time.output, tg.template getInput<0>());
	connect(reference.output, tg.template getInput<1>());
	connect(wam.jpOutput, tg.template getInput<2>());
	connect(wam.jtSum.output, tg.template getInput<3>());

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
	reference.setValue(endPos);
	sleep(2);


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
