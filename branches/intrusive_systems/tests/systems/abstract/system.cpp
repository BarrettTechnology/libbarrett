/*
 * system.cpp
 *
 *  Created on: Sep 19, 2009
 *      Author: dc
 */

#include <vector>
#include <iostream>
#include <gtest/gtest.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/manual_execution_manager.h>
#include "../exposed_io_system.h"


namespace {
using namespace barrett;


const double T_s = 0.002;
class SystemTest : public ::testing::Test {
public:
	SystemTest() : mem(T_s) {
		mem.startManaging(in);
	}

protected:
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> out;
	ExposedIOSystem<double> in;
};


TEST_F(SystemTest, GeneralIO) {
	systems::connect(out.output, in.input);
	EXPECT_FALSE(in.inputValueDefined()) << "input value not initially undefined";

	// set outputValue, then make sure value is defined and correct
	checkConnected(mem, &out, in, 12.2);

	out.setOutputValueUndefined();
	EXPECT_FALSE(in.inputValueDefined())
		<< "input value defined after call to outputValue.setUndefined()";

	// set outputValue, then make sure value is defined and correct
	checkConnected(mem, &out, in, 145.0);
}

TEST_F(SystemTest, OutputNotifyInputs) {
	const size_t numInputs = 50;

	std::vector<ExposedIOSystem<double>*> systems(numInputs);
	for (size_t i = 0; i < numInputs; ++i) {
		systems[i] = new ExposedIOSystem<double>;
		systems::connect(out.output, systems[i]->input);
		systems[i]->operateCalled = false;
	}

	out.setOutputValue(-87.1);

	for (size_t i = 0; i < numInputs; ++i) {
		out.operateCalled = false;
		systems[i]->inputValueDefined();  // update the value
		EXPECT_TRUE(out.operateCalled)
			<< "operate() didn't get called on EIOS for system " << i;

		delete systems[i];
	}
}

// TODO(dc): the delegate system could be better tested
TEST_F(SystemTest, OutputDelegates) {
	ExposedIOSystem<double> d;
	out.delegateOutputValueTo(d.output);

	systems::connect(out.output, in.input);
	checkConnected(mem, &d, in, 34.8);
}

TEST_F(SystemTest, OutputDelegatesCanBeChained) {
	ExposedIOSystem<double> d1, d2;
	d2.delegateOutputValueTo(d1.output);
	out.delegateOutputValueTo(d2.output);

	systems::connect(out.output, in.input);
	checkConnected(mem, &d1, in, 38.234);
}


// death tests
typedef SystemTest SystemDeathTest;

TEST_F(SystemDeathTest, InputGetValueDiesWhenNotConnected) {
	ASSERT_DEATH(in.getInputValue(), "");
}

TEST_F(SystemDeathTest, InputGetValueDiesWhenUndefined) {
	systems::connect(out.output, in.input);

	ASSERT_DEATH(in.getInputValue(), "");
}


}
