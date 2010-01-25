/*
 * tool_orientation.cpp
 *
 *  Created on: Jan 22, 2010
 *      Author: dc
 */


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <barrett/systems/kinematics_base.h>
#include <barrett/systems/tool_orientation.h>
#include <barrett/systems/tool_orientation_controller.h>
#include "exposed_io_system.h"


namespace {
using namespace barrett;


const size_t DOF = 7;
typedef units::JointPositions<DOF> jp_type;
typedef units::JointVelocities<DOF> jv_type;


// TODO(dc): actually test this!
TEST(ToolOrientationTest, Blah) {
	struct config_t config;
	char filename[] = "test.config";  // in project directory
	config_init(&config);
	int err = config_read_file(&config,filename);
	if (err != CONFIG_TRUE) {
		config_destroy(&config);
		throw(std::runtime_error("Couldn't load test.config for the Kinematics test."));
	}

	config_setting_t* wamconfig = config_lookup(&config, "wam");


	jp_type jp;
	jv_type jv;

	jp <<  0, -2, 0, 3.14, 0, 1.57, 0;
	jv.assign(0.0);

	systems::Constant<jp_type> jpSys(jp);
	systems::Constant<jv_type> jvSys(jv);
	systems::Constant<Eigen::Quaterniond> qSys(Eigen::Quaterniond(0.0, 0.6*sin(M_PI_4), 0.8*sin(M_PI_4), cos(M_PI_4)));

	systems::KinematicsBase<DOF> kinSys(config_setting_get_member(wamconfig, "kinematics"));
	systems::ToolOrientation<DOF> toSys;
	systems::ToolOrientationController<DOF> tocSys;
	ExposedIOSystem<units::JointTorques<DOF> > eios;


	config_destroy(&config);


	systems::connect(jpSys.output, kinSys.jpInput);
	systems::connect(jvSys.output, kinSys.jvInput);
	systems::connect(kinSys.kinOutput, toSys.kinInput);
	systems::connect(kinSys.kinOutput, tocSys.kinInput);

	systems::connect(qSys.output, tocSys.referenceInput);
	systems::connect(toSys.output, tocSys.feedbackInput);
	systems::connect(tocSys.controlOutput, eios.input);

//	eios.inputValueDefined();
	std::cout << eios.getInputValue();
}


}
