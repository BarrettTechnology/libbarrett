/*
 * log_temp_data.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: dc
 */

#include <vector>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// Disable torque saturation
	wam.jpController.setControlSignalLimit(v_type(0.0));


	jp_type tmp;
	std::vector<jp_type> poses;

	// J1
	tmp.setZero();
	tmp[0] = -2.6;
	tmp[1] = -M_PI_2;
	tmp[3] = M_PI_2;
	poses.push_back(tmp);
	tmp[0] = -tmp[0];
	poses.push_back(tmp);

	// J2
	tmp.setZero();
	tmp[1] = -1.9;
	poses.push_back(tmp);
	tmp[1] = -tmp[1];
	poses.push_back(tmp);

	// J3
	tmp.setZero();
	tmp[1] = M_PI_2;
	tmp[2] = -2.5;
	tmp[3] = M_PI_2;
	poses.push_back(tmp);
	tmp[2] = -tmp[2];
	poses.push_back(tmp);

	// J4 forward curls
	tmp.setZero();
	tmp[1] = -M_PI_2;
	tmp[3] = -0.9;
	poses.push_back(tmp);
	tmp[3] = 3.1;
	if (DOF >= 6) {
		tmp[5] = -0.7;
	}
	poses.push_back(tmp);

	// J4 backward curls
	tmp.setZero();
	tmp[1] = M_PI_2;
	tmp[3] = -0.9;
	poses.push_back(tmp);
	tmp[3] = M_PI_2;
	poses.push_back(tmp);


	size_t i;
	jp_type a, b;
	jp_type prev = wam.getJointPositions();
	for (size_t reps = 0; reps < 30; ++reps) {
		i = 0;
		while (i < poses.size()) {
			a = poses[i++];
			b = poses[i++];
			for (size_t j = 0; j < 4; ++j) {
				wam.moveTo(prev, jv_type(), a, true, 0.5, 0.5);
				prev = a;
				wam.moveTo(prev, jv_type(), b, true, 0.5, 0.5);
				prev = b;
			}
			wam.moveTo(prev, jv_type(), wam.getHomePosition(), true, 0.5, 0.5);
			prev = wam.getHomePosition();
		}
	}


	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
