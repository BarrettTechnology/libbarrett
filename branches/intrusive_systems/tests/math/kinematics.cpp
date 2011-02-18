/*
 * kinematics.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#include <libconfig.h++>

#include <boost/tuple/tuple.hpp>
#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/math/kinematics.h>
#include <barrett/systems/constant.h>
#include <barrett/systems/tuple_grouper.h>
#include <barrett/systems/callback.h>
#include <barrett/systems/helpers.h>
//#include <barrett/systems/>
#include "../systems/exposed_io_system.h"


// TODO(dc): actually test this


namespace {
using namespace barrett;


const size_t DOF = 7;
BARRETT_UNITS_TYPEDEFS(DOF);


class KimematicsTest : public ::testing::Test {
public:
	KimematicsTest() :
		kin(NULL)
	{
		libconfig::Config config;
		config.readFile("test.config");
		kin = new math::Kinematics<DOF>(config.lookup("wam.kinematics"));
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

	jp << 7.30467e-05, -1.96708, -0.000456121, 3.04257, -0.0461776, 1.54314, -0.0226513;
	jv.setConstant(0.0);

//	EXPECT_EQ(units::CartesianPosition::type(), (*kin)(boost::make_tuple(jp, jv)));
}

TEST_F(KimematicsTest, Stuff) {
	jp_type jp;
	jv_type jv;

	jp <<  0, -2, 0, 3.14, 0, 1.57, 0;
	jv.setConstant(0.0);

	(*kin)(boost::make_tuple(jp, jv));
	(*kin)(boost::make_tuple(jp, jv));


	systems::Constant<jp_type> jpSys(jp);
	systems::Constant<jv_type> jvSys(jv);
	systems::TupleGrouper<jp_type, jv_type> kinTg;
	systems::Callback<boost::tuple<jp_type, jv_type>, units::CartesianPosition::type> kinSys(boost::ref(*kin));
	ExposedIOSystem<units::CartesianPosition::type> eios;

	systems::connect(jpSys.output, kinTg.getInput<0>());
	systems::connect(jvSys.output, kinTg.getInput<1>());
	systems::connect(kinTg.output, kinSys.input);
	systems::connect(kinSys.output, eios.input);

	eios.getInputValue();
	(*kin)(boost::make_tuple(jp, jv));


}



}
