/*
 * joint_torques.cpp
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */


#include <sstream>
#include <stdexcept>
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


void fillJT(units::JointTorques* jt) {
	for (size_t i = 0; i < jt->size(); ++i) {
		jt->at(i) = i/2.0 - 4.0;  // store some non-default values
	}
}


TEST_F(JointTorquesTest, DefaultSize) {
	EXPECT_EQ(units::DOF, jt.size());
}

TEST_F(JointTorquesTest, DOFIsRuntimeAdjustable) {
	size_t origDOF = units::DOF;
	units::DOF += 5;

	units::JointTorques jt2;
	EXPECT_EQ(units::DOF, jt2.size());

	units::DOF = origDOF;
}

TEST_F(JointTorquesTest, AccessAndModifyMembersByIndex) {
	for (size_t i = 0; i < jt.size(); ++i) {
		jt.at(i) = i/10.0 - 1;  // test both at()
		EXPECT_EQ(i/10.0 - 1, jt[i]);  // and []
	}
}

TEST_F(JointTorquesTest, AccessAndModifyMembersByIterator) {
	units::JointTorques::iterator i;
	for (i = jt.begin(); i != jt.end(); ++i) {
		*i = -38.1e7;
		EXPECT_EQ(-38.1e7, *i);
	}
}

TEST_F(JointTorquesTest, InitialValue) {
	for (size_t i = 0; i < jt.size(); ++i) {
		EXPECT_EQ(0.0, jt[i]);
	}
}

TEST_F(JointTorquesTest, CopyFromJointTorques) {
	fillJT(&jt);
	units::JointTorques jt_copy = jt;  // uses copy constructor

	for (size_t i = 0; i < jt.size(); ++i) {
		EXPECT_EQ(jt[i], jt_copy[i]);
	}
}

TEST_F(JointTorquesTest, AssignmentFromJointTorques) {
	fillJT(&jt);
	units::JointTorques jt_copy;
	jt_copy = jt;  // uses assignment operator

	for (size_t i = 0; i < jt.size(); ++i) {
		EXPECT_EQ(jt[i], jt_copy[i]);
	}
}

TEST_F(JointTorquesTest, CopyFromArray) {
	double jt_array[units::DOF];
	for (size_t i = 0; i < units::DOF; ++i) {
		jt_array[i] = i/10.0;
	}

	units::JointTorques jt_copy = jt_array;

	for (size_t i = 0; i < jt.size(); ++i) {
		EXPECT_EQ(jt_array[i], jt_copy[i]);
	}
}

TEST_F(JointTorquesTest, AssignmentFromArray) {
	double jt_array[units::DOF];
	for (size_t i = 0; i < units::DOF; ++i) {
		jt_array[i] = i/10.0;
	}

	units::JointTorques jt_copy;
	jt_copy = jt_array;

	for (size_t i = 0; i < jt.size(); ++i) {
		EXPECT_EQ(jt_array[i], jt_copy[i]);
	}
}

TEST_F(JointTorquesTest, EqualityOperators) {
	double jt_array[units::DOF];
	for (size_t i = 0; i < units::DOF; ++i) {
		jt_array[i] = i/10.0;
	}

	units::JointTorques jt_copy;
	EXPECT_TRUE(jt == jt_copy);
	EXPECT_FALSE(jt != jt_copy);

	jt_copy = jt_array;
	jt = jt_array;
	EXPECT_TRUE(jt == jt_copy);
	EXPECT_FALSE(jt != jt_copy);

	double tmp;
	for (size_t i = 0; i < jt.size(); ++i) {
		tmp = jt[i];
		jt[i] = -4008.7;

		EXPECT_FALSE(jt == jt_copy);
		EXPECT_TRUE(jt != jt_copy);

		jt[i] = tmp;
	}
}

TEST_F(JointTorquesTest, EqualityOperatorsThrow) {
	size_t origDOF = units::DOF;
	units::DOF += 5;
	units::JointTorques jt2;

	EXPECT_THROW(jt == jt2, std::invalid_argument);
	EXPECT_THROW(jt != jt2, std::invalid_argument);

	units::DOF = origDOF;
}

TEST_F(JointTorquesTest, OstreamOperator) {
	size_t origDOF = units::DOF;
	units::DOF = 3;
	units::JointTorques jt2;

	std::stringstream strs;
	strs << jt2;
	EXPECT_EQ("[0, 0, 0]", strs.str());

	units::DOF = origDOF;
}


}
