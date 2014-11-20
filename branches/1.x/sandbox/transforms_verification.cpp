/**
 * @file transforms_verification.cpp
 *
 * @date Feb 20, 2014
 * @author Jonathan Hagstrand
 *
 */

#include <iostream>
#include <stdlib.h>
#include <barrett/units.h>
#include <barrett/math.h>
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

	sqm_type J2MP,M2JP,J2MT,J2PP,P2JP,J2PT;
	v_type JE2JP;

	JE2JP = wam.getLowLevelWam().getJointEncoderToJointPositionTransform();
	std::cout << "Joint Encoder to Joint Position Transform " << JE2JP << std::endl;
	J2MP = wam.getLowLevelWam().getJointToMotorPositionTransform();
	std::cout << "Joint to Motor Position Transform " << J2MP << std::endl;
	M2JP = wam.getLowLevelWam().getMotorToJointPositionTransform();
	std::cout << "Motor to Joint Position Transform " << M2JP << std::endl;
	J2MT = wam.getLowLevelWam().getJointToMotorTorqueTransform();
	std::cout << "Joint to Motor Torque Transform " << J2MT << std::endl;
	J2PP = wam.getLowLevelWam().getJointToPuckPositionTransform();
	std::cout << "Joint to Puck Position Transform " << J2PP << std::endl;
	P2JP = wam.getLowLevelWam().getPuckToJointPositionTransform();
	std::cout << "Puck to Joint Position Transform " << P2JP << std::endl;
	J2PT = wam.getLowLevelWam().getJointToPuckTorqueTransform();
	std::cout << "Joint to Puck Torque Transform " << J2PT << std::endl;

	return 0;
}

/*
 *  const sqm_type& getJointToMotorPositionTransform() const { return j2mp; }
	const sqm_type& getMotorToJointPositionTransform() const { return m2jp; }
	const sqm_type& getJointToMotorTorqueTransform() const { return j2mt; }

	const sqm_type& getJointToPuckPositionTransform() const { return j2pp; }
	const sqm_type& getPuckToJointPositionTransform() const { return p2jp; }
	const sqm_type& getJointToPuckTorqueTransform() const { return j2pt; }
	const v_type& getJointEncoderToJointPositionTransform() const { return jointEncoder2jp; }
 *
 */
