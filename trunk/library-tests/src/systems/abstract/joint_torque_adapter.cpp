/*
 * joint_torque_adapter.cpp
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <gtest/gtest.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/joint_torque_adapter.h>

#include "../exposed_io_system.h"


namespace {
using namespace barrett;


class JTA : public systems::JointTorqueAdapter {
// IO
public:		Input<units::JointTorques> controlInput;


public:
	JTA() :
		systems::JointTorqueAdapter(), controlInput(this) {}
	explicit JTA(const units::JointTorques& initialControlOutputValue) :
		systems::JointTorqueAdapter(initialControlOutputValue),
		controlInput(this) {}
	virtual ~JTA() {}

	// a JointTorques input to make testing simpler
	virtual System::Input<units::JointTorques>* getControlInput() {
		return &controlInput;
	}

protected:
	virtual void operate() {}

private:
	DISALLOW_COPY_AND_ASSIGN(JTA);
};


// we just want this to compile
TEST(JointTorqueAdapterTest, Interface) {
	JTA jta;

	jta.getControlInput();
	systems::System::Output<units::JointTorques>& jto = jta.jointTorqueOutput;

	// do something with jto so we don't get an unused variable warning
	systems::connect(jto, jta.controlInput);
}

TEST(JointTorqueAdapterTest, InitialOutputValueCtor) {
	double jt_array[units::DOF];
	for (size_t i = 0; i < units::DOF; ++i) {
		jt_array[i] = i/10.0;
	}
	units::JointTorques jt = jt_array;

	JTA jta(jt);
	ExposedIOSystem<units::JointTorques> eios;
	systems::connect(jta.jointTorqueOutput, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "initial value undefined";
	EXPECT_EQ(jt, eios.getInputValue())
		<< "wrong initial value given";
}


}
