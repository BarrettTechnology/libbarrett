/*
 * kinematics.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#include <libconfig.h>

#include <boost/tuple/tuple.hpp>
#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/math/kinematics.h>
#include <barrett/systems.h>
#include "../systems/exposed_io_system.h"


namespace {
using namespace barrett;


const size_t DOF = 7;
typedef units::JointPositions<DOF> jp_type;
typedef units::JointVelocities<DOF> jv_type;


class KimematicsTest : public ::testing::Test {
public:
	KimematicsTest() :
		kin(NULL)
	{
		struct config_t config;
		char filename[] = "test.config";  // in project directory
		config_init(&config);
		int err = config_read_file(&config,filename);
		if (err != CONFIG_TRUE) {
			config_destroy(&config);
			throw(std::runtime_error("Couldn't load test.config for the Kinematics test."));
		}

		config_setting_t* wamconfig = config_lookup(&config, "wam");
		kin = new math::Kinematics<DOF>(config_setting_get_member(wamconfig, "kinematics"));

		config_destroy(&config);
	}

	~KimematicsTest() {
		delete kin;
		kin = NULL;
	}

protected:
	math::Kinematics<DOF>* kin;
};


TEST_F(KimematicsTest, Ctor) {
	ASSERT_TRUE(kin->impl != NULL);
	EXPECT_EQ(DOF, kin->impl->dof);
}

TEST_F(KimematicsTest, Eval) {
	jp_type jp;
	jv_type jv;

//	jp <<  0, -2, 0, 3.14, 0, 1.57, 0;
	jp << 7.30467e-05, -1.96708, -0.000456121, 3.04257, -0.0461776, 1.54314, -0.0226513;
	jv.assign(0.0);

//	kin->eval(jp, jv);
//	EXPECT_EQ(units::CartesianPosition(), units::CartesianPosition(kin->impl->tool->origin_pos));
	EXPECT_EQ(units::CartesianPosition(), (*kin)(boost::make_tuple(jp, jv)));
}

TEST_F(KimematicsTest, Stuff) {
	jp_type jp;
	jv_type jv;

	jp <<  0, -2, 0, 3.14, 0, 1.57, 0;
//	jp << 7.30467e-05, -1.96708, -0.000456121, 3.04257, -0.0461776, 1.54314, -0.0226513;
	jv.assign(0.0);

	(*kin)(boost::make_tuple(jp, jv));
	(*kin)(boost::make_tuple(jp, jv));


	systems::Constant<jp_type> jpSys(jp);
	systems::Constant<jv_type> jvSys(jv);
	systems::TupleGrouper<jp_type, jv_type> kinTg;
	systems::Callback<boost::tuple<jp_type, jv_type>, units::CartesianPosition> kinSys(boost::ref(*kin));
//	systems::PrintToStream<units::CartesianPosition> printCpos;
	ExposedIOSystem<units::CartesianPosition> eios;

	systems::connect(jpSys.output, kinTg.getInput<0>());
	systems::connect(jvSys.output, kinTg.getInput<1>());
	systems::connect(kinTg.output, kinSys.input);
//	systems::connect(kinSys.output, printCpos.input);
	systems::connect(kinSys.output, eios.input);

	eios.getInputValue();
	(*kin)(boost::make_tuple(jp, jv));


}



}
