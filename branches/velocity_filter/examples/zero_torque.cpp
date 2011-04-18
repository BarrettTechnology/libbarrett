#include <boost/tuple/tuple.hpp>

#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_CONFIGURE_PM
#include <barrett/standard_main_function.h>


using namespace barrett;

const char BIN_FILE[] = "j4_step.bin";
const char CSV_FILE[] = "j4_step.csv";


template <size_t DOF>
typename units::JointTorques<DOF>::type calcJt(double time) {
	typename units::JointTorques<DOF>::type jt(0.0);

	// step
//	jt[3] = -4.0;

	// sine
	jt[3] = -2*sin(time * 4.0);

	return jt;
}

template <size_t DOF>
typename units::JointPositions<DOF>::type calcJp(double time) {
	typename units::JointPositions<DOF>::type jp(0.0);
	jp[1] = -M_PI_2;
	jp[2] = -M_PI_2;

	// sine
	jp[3] = 1.0 - 1*sin(1*time);

	return jp;
}


bool configure_pm(int argc, char** argv, ProductManager& pm) {
	pm.getExecutionManager(0.001);
	return true;
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	systems::ExecutionManager* em = pm.getExecutionManager();

	typedef boost::tuple<jt_type, jp_type, jv_type, ja_type> tuple_type;

	systems::TupleGrouper<jt_type, jp_type, jv_type, ja_type> tg;
	systems::PeriodicDataLogger<tuple_type> logger(em,
			new log::RealTimeWriter<tuple_type>(BIN_FILE, em->getPeriod()),
			1);

	systems::connect(wam.jtSum.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jvOutput, tg.template getInput<2>());
	systems::connect(wam.jaOutput, tg.template getInput<3>());
	systems::connect(tg.output, logger.input);


//	wam.gravityCompensate();
	usleep(250000);

	jp_type startPos(0.0);
	startPos[1] = -M_PI_2;
	startPos[2] = -M_PI_2;
//	startPos[3] = M_PI;
//	startPos[3] = M_PI - 0.2;
	startPos[3] = 1.0;
	wam.moveTo(startPos);

	wam.jpController.getKp()[3] = 0.0;
	wam.jpController.getKi()[3] = 0.0;
	wam.jpController.getKd()[3] = 0.0;
	wam.jpController.resetIntegrator();
	sleep(1);

	// adjust velocity fault limit
	pm.getSafetyModule()->getPuck()->setProperty(Puck::VL2, (int)(1.5*0x1000));

//	jt_type jt(0.0);
//	jt[3] = -4.0;
//	systems::ExposedOutput<jt_type> eo(jt);
//	systems::connect(eo.output, wam.input);

	systems::Ramp time(em);

	systems::Callback<double, jt_type> jtCalc(calcJt<DOF>);
	systems::connect(time.output, jtCalc.input);

//	systems::Callback<double, jp_type> jpCalc(calcJp<DOF>);
//	systems::connect(time.output, jpCalc.input);

	{
		BARRETT_SCOPED_LOCK(em->getMutex());

		systems::connect(jtCalc.output, wam.input);
//		wam.trackReferenceSignal(jpCalc.output);
		time.start();
	}

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	pm.getExecutionManager()->stop();

	logger.closeLog();
	log::Reader<tuple_type> reader(BIN_FILE);
	reader.exportCSV(CSV_FILE);

	return 0;
}
