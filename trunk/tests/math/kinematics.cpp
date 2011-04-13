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


// TODO(dc): actually test this


namespace {
using namespace barrett;


const size_t DOF = 7;
BARRETT_UNITS_TYPEDEFS(DOF);


class KinematicsTest : public ::testing::Test {
public:
	KinematicsTest() :
		kin(NULL)
	{
		libconfig::Config config;
		config.readFile("test.config");
		kin = new math::Kinematics<DOF>(config.lookup("wam.kinematics"));
	}

	~KinematicsTest() {
		delete kin;
		kin = NULL;
	}

protected:
	math::Kinematics<DOF>* kin;
};


TEST_F(KinematicsTest, Ctor) {
	ASSERT_TRUE(kin->impl != NULL);
	EXPECT_EQ(DOF, kin->impl->dof);
}

//TEST_F(KinematicsTest, Eval) {
//	jp_type jp;
//	jv_type jv;
//
//	jp << 7.30467e-05, -1.96708, -0.000456121, 3.04257, -0.0461776, 1.54314, -0.0226513;
//	jv.setConstant(0.0);
//
//	EXPECT_EQ(units::CartesianPosition::type(), (*kin)(boost::make_tuple(jp, jv)));
//
//	jp.setConstant(0.0);
//	EXPECT_EQ(units::CartesianPosition::type(), (*kin)(boost::make_tuple(jp, jv)));
//}


}
