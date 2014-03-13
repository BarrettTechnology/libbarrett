/**
 * bow_and_arrow.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: Jonathan Hagstrand
 *
 *  This demo mimics shooting a bow and arrow. The user will set the 'home' or origin position
 *  of the bow to allow for more realistic movement for drawing the string. The intended application
 *  will allow the user to pull the draw string any way they like away from the origin and simulate a
 *  force directly back to the origin. This should allow the user to aim high/low/straight and give the
 *  same weighted feel of the string.
 *
 *  Bow and Arrow Equation will follow such dyanmics:
 *     Potential Energy = Kinetic Energy
 *     (.05)*m*v^2 + k*(.5)*m*v^2 = (.5)*e*F*(delta position)
 *     m = mass of bow
 *     v = velocity
 *     e = efficiency ratio
 *     F = Force (Newtons)
 *     k = bow flexion coefficient
 *     delta position = home - drawn cartesian positions (x,y,z)
 *
 *  Currently implemented Linear Lookup for time being: F = dW / dL(30" or
 *
 *  Commandline: ./bow_and_arrow <drawWeight in ft-lbs> <drawWLength in inches>
 */

#include <iostream>
#include <stdlib.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/pid_controller.h>
#include <barrett/os.h>
#include <barrett/systems.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;
using systems::connect;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	double dW,dL,bF;
	if(argc == 3){
		dL = atof(argv[2]);
		dL = dL*0.0254;
		dW = atof(argv[1]);
	}
	else if(argc == 2){
		dW = atof(argv[1]);
		dL = 0.762;
	}else{
		dW = 30.0;
		dL = 0.762;
	}

	bF = (dW / dL)*1.35581795;
	// Lets prevent the torque fault by limiting acceleration?
	//pm.safetyModule->setVelocityLimit(2.0);

	// Lets instantiate the system calls
	systems::ExposedOutput<cp_type> homePos;
	systems::Summer<cp_type> posErr("+-");
	systems::Gain<cp_type, double, cf_type> gain(bF);
	systems::ToolForceToJointTorques<DOF> tf2jt;

	connect(homePos.output, posErr.getInput(0));
	connect(wam.toolPosition.output, posErr.getInput(1));

	connect(posErr.output, gain.input);

	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(gain.output, tf2jt.input);

	homePos.setValueUndefined();
	connect(tf2jt.output, wam.input);

	wam.gravityCompensate();
	/*
	std::string line;
	bool active = true,set=false;
	while(active){
		std::getline(std::cin, line);
		switch (line[0]) {
			case 'x':
			case 'q':
				active = false;
				break;
			default:
				if (line.size() != 0) {
					if(set){
						homePos.setValue(wam.getToolPosition());
					}else{
						homePos.setValueUndefined();
					}
					set = !set;
				}
				break;
		}
	}*/

	while(pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE){
		waitForEnter();
		homePos.setValue(wam.getToolPosition());
		printf(">>> Bow Position Locked.");

		waitForEnter();
		homePos.setValueUndefined();
		printf(">>> Bow Position Unlocked.");
	}

	wam.idle();
//	wam.moveHome();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}


// End of Main
