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
class FeedForward : public systems::SingleIO<typename units::JointPositions<DOF>::type, typename units::JointTorques<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	bool going;

public:
	explicit FeedForward() : going(false) {}

	void start() {
		going = true;
	}

protected:
	jt_type data;

	virtual void operate() {
		if (going) {
			double angle = this->input.getValue()[3];
			if (angle > 1.95) {
				data[3] = -10.0;
			} else if (angle < 1.8) {
				data[3] = 0.0;
				going = false;
			} else {
				data[3] = 8.0;
			}
		} else {
			data[3] = 0.0;
		}

		this->outputValue->setData(&data);
	}
};

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


	wam.jpController.setControlSignalLimit(jt_type(0.0));  // Turn off saturation

//	systems::PIDController<jp_type, ja_type> pid(config.lookup("control_joint"));
//	systems::InverseDynamics<DOF> id(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["dynamics"]);
//	connect(wam.jpOutput, pid.feedbackInput);
//	connect(wam.jvOutput, id.jvInput);
//	connect(wam.kinematicsBase.kinOutput, id.kinInput);
//	connect(pid.controlOutput, id.input);
//	wam.supervisoryController.registerConversion(systems::makeIOConversion(pid.referenceInput, id.output));


//	FeedForward<DOF> ff;
//	connect(wam.jpOutput, ff.input);
//	connect(ff.output, wam.input);


	wam.gravityCompensate();
	usleep(250000);

//	detail::waitForEnter();
//	wam.moveTo(jp_type(0.0));
//	detail::waitForEnter();
//	wam.moveTo(jp_type(1.0));
//	detail::waitForEnter();
//	wam.moveHome();
//	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
//	return 0;

	wam.moveTo(startPos);

//	systems::ExposedOutput<jp_type> reference(startPos);
	std::vector<jp_type> vec;
	vec.push_back(startPos);
	vec.push_back(endPos);
	math::Spline<jp_type> spline(vec);
	math::TrapezoidalVelocityProfile profile(5.0, 100.0, 0.0, spline.changeInS());
//	math::TrapezoidalVelocityProfile profile(0.5, 0.5, 0.0, spline.changeInS());

	systems::Ramp moveTime(pm.getExecutionManager(), 1.0);
	systems::Callback<double, jp_type> trajectory(boost::bind(boost::ref(spline), boost::bind(boost::ref(profile), _1)));

	connect(moveTime.output, trajectory.input);
	wam.trackReferenceSignal(trajectory.output);


	double omega_p = 1000.0;
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
	sleep(1);

	connect(changeUnits.output, id.input);
	connect(changeUnits.output, driveInertias.input);
	connect(wam.jvOutput, id.jvInput);
	connect(wam.kinematicsBase.kinOutput, id.kinInput);
	connect(id.output, idSum.getInput(0));
	connect(driveInertias.output, idSum.getInput(1));
	connect(idSum.output, wam.input);

	wam.jpController.getKp() *= 1.0;
	wam.jpController.getKi() *= 1.0;
	wam.jpController.getKd() *= 1.0;

//	wam.trackReferenceSignal(reference.output);
	sleep(2);
//	wam.jpController.getKp()[1] = 0.0;
//	wam.jpController.getKi()[1] = 0.0;
//	wam.jpController.getKd()[1] = 0.0;


	systems::Ramp time(pm.getExecutionManager(), 1.0);

	systems::TupleGrouper<double, jp_type, jp_type, jt_type> tg;
	connect(time.output, tg.template getInput<0>());
	connect(trajectory.output, tg.template getInput<1>());
	connect(wam.jpOutput, tg.template getInput<2>());
	connect(wam.jtSum.output, tg.template getInput<3>());
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
//	wam.idle();
//	ff.start();
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
