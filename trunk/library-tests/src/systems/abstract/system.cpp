/*
 * system.cpp
 *
 *  Created on: Sep 19, 2009
 *      Author: dc
 */

#include <vector>
#include <iostream>
#include <gtest/gtest.h>
#include <barrett/systems.h>
#include "../exposed_io_system.h"


namespace {
using namespace barrett;


// TODO(dc): make this a type-parameterized test case so users can test their
// own code
class SystemTest : public ::testing::Test {
protected:
	ExposedIOSystem<double> eios;
};


TEST_F(SystemTest, GeneralIO) {
	systems::connect(eios.output, eios.input);
	EXPECT_FALSE(eios.inputValueDefined())
		<< "input value not initially undefined when no initial value given";

	// set outputValue, then make sure value is defined and correct
	checkConnected(&eios, eios, 12.2);

	eios.setOutputValueUndefined();
	EXPECT_FALSE(eios.inputValueDefined())
		<< "input value defined after call to outputValue.setValueUndefined()";

	// set outputValue, then make sure value is defined and correct
	checkConnected(&eios, eios, 145.0);
}

TEST_F(SystemTest, InputGetValueThrowsWhenNotConnected) {
	EXPECT_THROW(eios.getInputValue(), std::logic_error)
		<< "input.getValue() didn't throw when not connected";
}

TEST_F(SystemTest, InputGetValueThrowsWhenUndefined) {
	systems::connect(eios.output, eios.input);

	EXPECT_THROW(eios.getInputValue(),
			std::logic_error)
		<< "input.getValue() didn't throw when value undefined";
}

TEST_F(SystemTest, OutputInitialValueCtor) {
	systems::System::Output<double>::Value* outputValue;
	systems::System::Output<double> output(38.12, &outputValue);
	systems::connect(output, eios.input);

	EXPECT_TRUE(eios.inputValueDefined())
		<< "input undefined when connected to output with initial value";
	EXPECT_EQ(38.12, eios.getInputValue())
		<< "input has wrong value when connected to output with initial value";
}

TEST_F(SystemTest, OutputNotifyInputs) {
	const size_t numInputs = 50;

	std::vector<ExposedIOSystem<double>*> systems(numInputs);
	for (size_t i = 0; i < numInputs; ++i) {
		systems[i] = new ExposedIOSystem<double>;
		systems::connect(eios.output, systems[i]->input);
		systems[i]->operateCalled = false;
	}

	eios.setOutputValue(-87.1);

	for (size_t i = 0; i < numInputs; ++i) {
		EXPECT_TRUE(systems[i]->operateCalled)
			<< "operate() didn't get called on system " << i;

		delete systems[i];
	}
}

// TODO(dc): the delegate system could be better tested
TEST_F(SystemTest, OutputDelegates) {
	ExposedIOSystem<double> d;
	eios.delegateOutputValueTo(d.output);

	systems::connect(eios.output, eios.input);
	checkConnected(&d, eios, 34.8);
}

TEST_F(SystemTest, OutputDelegatesCanBeChained) {
	ExposedIOSystem<double> d1, d2;
	d2.delegateOutputValueTo(d1.output);
	eios.delegateOutputValueTo(d2.output);

	systems::connect(eios.output, eios.input);
	checkConnected(&d1, eios, 38.234);
}


}
