/*
 * constrain_to_path.cpp
 *
 *  Created on: Jun 21, 2012
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;


cf_type scale(boost::tuple<cf_type, double> t) {
	return t.get<0>() * t.get<1>();
}

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(const typename units::JointTorques<DOF>::type& x, const typename units::JointTorques<DOF>::type& limit) {
	int index;
	double minRatio;

	minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
	if (minRatio < 1.0) {
		return minRatio * x;
	} else {
		return x;
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	pm.getSafetyModule()->setVelocityLimit(1.25);

	while (true) {
		pm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE);
		wam.gravityCompensate();

		do {
			{
				const double T_s = pm.getExecutionManager()->getPeriod();
				// Record at 1/10th of the loop rate
				systems::PeriodicDataLogger<cp_type> tpLogger(pm.getExecutionManager(),
						new barrett::log::RealTimeWriter<cp_type>(tmpFile, 10*T_s), 10);


				printf("Press [Enter] to start recording path.\n");
				waitForEnter();
				connect(wam.toolPosition.output, tpLogger.input);

				printf("Press [Enter] to stop recording.\n");
				waitForEnter();
				wam.moveTo(wam.getToolPosition());  // Hold position at one end point
				tpLogger.closeLog();
			}

			// Build spline between recorded points
			log::Reader<cp_type> lr(tmpFile);
			std::vector<cp_type> vec;
			for (size_t i = 0; i < lr.numRecords(); ++i) {
				vec.push_back(lr.getRecord());
			}
			systems::HapticPath hp(vec);


			systems::PIDController<double, double> comp;
			systems::Constant<double> zero(0.0);
			systems::TupleGrouper<cf_type, double> tg;
			systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(scale);
			systems::Gain<cp_type, double, cf_type> tangentGain(0.0);
			systems::Summer<cf_type> tfSum;
			systems::ToolForceToJointTorques<DOF> tf2jt;

			jt_type jtLimits(35.0);
			systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits));

			// configure Systems
			double kp = 3e3;
			double kd = 3e1;

			comp.setKp(kp);
			comp.setKd(kd);

			// connect Systems
			connect(wam.toolPosition.output, hp.input);

			connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
			connect(hp.directionOutput, tg.getInput<0>());

			connect(hp.depthOutput, comp.referenceInput);
			connect(zero.output, comp.feedbackInput);
			connect(comp.controlOutput, tg.getInput<1>());

			connect(tg.output, mult.input);
			connect(mult.output, tfSum.getInput(0));

			connect(hp.tangentDirectionOutput, tangentGain.input);
			connect(tangentGain.output, tfSum.getInput(1));

			connect(tfSum.output, tf2jt.input);
			connect(tf2jt.output, jtSat.input);

			connect(jtSat.output, wam.input);

			wam.idle();


			std::string line;
			while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
				printf(">>> Set tangent force ('x' to record another path): ");
				std::getline(std::cin, line);

				if (line[0] == 'x') {
					break;
				}

				double f = strtod(line.c_str(), NULL);
				tangentGain.setGain(f);
			}
		} while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE);

		wam.gravityCompensate(false);
	}

	std::remove(tmpFile);
	return 0;
}
