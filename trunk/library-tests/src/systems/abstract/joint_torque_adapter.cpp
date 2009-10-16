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
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/joint_torque_adapter.h>

#include "../exposed_io_system.h"


namespace {
using namespace barrett;


class JTA : public systems::JointTorqueAdapter {
// IO
public:		Input<std::vector<double> > controlInput;


public:
	JTA() :
		systems::JointTorqueAdapter(), controlInput(this) {}
	explicit JTA(const std::vector<double>& initialControlOutputValue) :
		systems::JointTorqueAdapter(initialControlOutputValue),
		controlInput(this) {}
	virtual ~JTA() {}

	virtual System::Input<std::vector<double> >* getControlInput() {
		return &controlInput;
	}

protected:
	virtual void operate() {}

private:
	DISALLOW_COPY_AND_ASSIGN(JTA);
};


// gtest doesn't compile if you make assertions about objects without ostream
// operators :(
std::ostream& operator<< (std::ostream& os, const std::vector<double>& seq) {
	os << "[";

	std::vector<double>::const_iterator i = seq.begin();
	while (i != seq.end()) {
		os << *i;
		if (++i != seq.end()) {
			os << ", ";
		}
	}

	os << "]";
	return os;
}


// we just want this to compile
TEST(JointTorqueAdapterTest, Interface) {
	JTA jta;

	jta.getControlInput();
	systems::System::Output<std::vector<double> >& jto = jta.jointTorqueOutput;

	// do something with jto so we don't get an unused variable warning
	systems::connect(jto, jta.controlInput);
}

TEST(JointTorqueAdapterTest, InitialOutputValueCtor) {
	double iv_array[] = {3.0, -7.8, 5e10};
	std::vector<double> iv_vector(iv_array,
			iv_array + sizeof(iv_array)/sizeof(iv_array[0]));

	JTA jta(iv_vector);
	ExposedIOSystem<std::vector<double> > eios;
	systems::connect(jta.jointTorqueOutput, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "initial value undefined";
	EXPECT_EQ(iv_vector, eios.getInputValue())
		<< "wrong initial value given";
}


}
