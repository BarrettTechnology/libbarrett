/*
 * tool_orientation.cpp
 *
 *  Created on: Jan 22, 2010
 *      Author: dc
 */


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <barrett/systems/helpers.h>
#include <barrett/systems/constant.h>
#include <barrett/systems/kinematics_base.h>
#include <barrett/systems/tool_orientation.h>
#include <barrett/systems/tool_orientation_controller.h>
#include <barrett/dynamics/dynamics.h>
#include <barrett/control/control_cartesian_xyz_q.h>
#include "exposed_io_system.h"


namespace {
using namespace barrett;


const size_t DOF = 7;
typedef units::JointPositions<DOF> jp_type;
typedef units::JointVelocities<DOF> jv_type;
typedef units::JointTorques<DOF> jt_type;


// TODO(dc): actually test this!
TEST(ToolOrientationTest, Blah) {
	struct config_t config;
	char filename[] = "test.config";  // in project directory
	config_init(&config);
	int err = config_read_file(&config,filename);
	ASSERT_EQ(CONFIG_TRUE, err);

	config_setting_t* wamconfig = config_lookup(&config, "wam");


	jp_type jp;
	jv_type jv;

//	jp <<  0, -2, 0, 3.14, 0, 1.57, 0;
	jp <<  1.21491, -1.96768, 0.0498996, 2.15371, 0.284814, 1.4271, 0.0467362;
	jv.assign(0.0);

	systems::Constant<jp_type> jpSys(jp);
	systems::Constant<jv_type> jvSys(jv);
//	systems::Constant<Eigen::Quaterniond> qSys(Eigen::Quaterniond(cos(M_PI_4), 0.0, 0.6*sin(M_PI_4), 0.8*sin(M_PI_4)));
	systems::Constant<Eigen::Quaterniond> qSys(Eigen::Quaterniond(0.517266, 0.453975, -0.528556, -0.496962));

	systems::KinematicsBase<DOF> kinSys(config_setting_get_member(wamconfig, "kinematics"));
	systems::ToolOrientation<DOF> toSys;
	systems::ToolOrientationController<DOF> tocSys;
	ExposedIOSystem<jt_type> eios;


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

TEST(ToolOrientationTest, Blah2) {
	struct config_t config;
	char filename[] = "test.config";  // in project directory
	config_init(&config);
	int err = config_read_file(&config,filename);
	ASSERT_EQ(CONFIG_TRUE, err);

	config_setting_t* wamconfig = config_lookup(&config, "wam");

	math::Kinematics<DOF> kin(config_setting_get_member(wamconfig, "kinematics"));

	struct bt_control_cartesian_xyz_q * con = NULL;
	bt_control_cartesian_xyz_q_create(&con,
			config_setting_get_member(wamconfig,
					"control_cartesian_xyz_q"),
			kin.impl, NULL);
	ASSERT_TRUE(con != NULL);


	jp_type jp;
	jv_type jv;
	jt_type jt;

//	jp <<  0, -2, 0, 3.14, 0, 1.57, 0;
	jp <<  1.21491, -1.96768, 0.0498996, 2.15371, 0.284814, 1.4271, 0.0467362;
	jv.assign(0.0);

	kin.eval(jp, jv);
	bt_control_get_position(&con->base);
	bt_control_hold(&con->base);

//	gsl_vector_set(con->ref_quat, 0, cos(M_PI_4));
//	gsl_vector_set(con->ref_quat, 1, 0.0);
//	gsl_vector_set(con->ref_quat, 2, 0.6*sin(M_PI_4));
//	gsl_vector_set(con->ref_quat, 3, 0.8*sin(M_PI_4));
	gsl_vector_set(con->ref_quat, 0, 0.517266);
	gsl_vector_set(con->ref_quat, 1, 0.453975);
	gsl_vector_set(con->ref_quat, 2, -0.528556);
	gsl_vector_set(con->ref_quat, 3, -0.496962);

	bt_control_eval(&con->base, jt.asGslVector(), 0.002);


	bt_control_cartesian_xyz_q_destroy(con);
	config_destroy(&config);

	std::cout << jt;
}


}
