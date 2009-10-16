/*
 * joint_torques.cpp
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/units.h>


namespace {
using namespace barrett;


class JointTorquesTest : public ::testing::Test {
public:
	JointTorquesTest() :
		jt() {}

protected:
	units::JointTorques jt;
};


/*
TEST_F(JointTorquesTest, DefaultSize) {
	EXPECT_EQ(units::DOF, jt.size());
}

TEST_F(JointTorquesTest, DOFIsRuntimeAdjustable) {
	size_t origDOF = units::DOF;
	units::DOF = 10;

	EXPECT_EQ(units::DOF, jt.size());

	units::DOF = origDOF;
}
*/

TEST_F(JointTorquesTest, AccessAndModifyMembers) {
	jt.resize(20);

	for (size_t i = 0; i < jt.size(); ++i) {
		jt.at(i) = i/10.0 - 1;
		EXPECT_EQ(i/10.0 - 1, jt[i]);
	}
}


}
