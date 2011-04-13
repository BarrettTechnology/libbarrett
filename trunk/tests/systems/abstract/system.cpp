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


class SystemTest : public ::testing::Test {
public:
	SystemTest() {
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

	mem.stopManaging(in);
	EXPECT_FALSE(in.inputValueDefined());
}

TEST_F(SystemTest, OnExecutionManagerChanged) {
	ExposedIOSystem<double> eios;
	ASSERT_FALSE(eios.executionManagerChanged);

	mem.startManaging(eios);
	EXPECT_TRUE(eios.executionManagerChanged);
	eios.executionManagerChanged = false;

	ASSERT_FALSE(out.executionManagerChanged);
	systems::connect(out.output, eios.input);
	EXPECT_FALSE(eios.executionManagerChanged);
	EXPECT_TRUE(out.executionManagerChanged);
	out.executionManagerChanged = false;

	mem.stopManaging(eios);
	EXPECT_TRUE(eios.executionManagerChanged);
	eios.executionManagerChanged = false;
	EXPECT_TRUE(out.executionManagerChanged);
	out.executionManagerChanged = false;

	mem.startManaging(eios);
	EXPECT_TRUE(eios.executionManagerChanged);
	eios.executionManagerChanged = false;
	EXPECT_TRUE(out.executionManagerChanged);
	out.executionManagerChanged = false;

	systems::disconnect(eios.input);
	EXPECT_FALSE(eios.executionManagerChanged);
	EXPECT_TRUE(out.executionManagerChanged);
	out.executionManagerChanged = false;
}

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

TEST_F(SystemTest, OutputDelegatePropagatesEmDirect) {
	ExposedIOSystem<double> d1, d2;

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_FALSE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d1.getExecutionManager());

	EXPECT_FALSE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d2.getExecutionManager());


	in.delegateOutputValueTo(d1.output);

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_TRUE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d1.getExecutionManager());

	EXPECT_FALSE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d2.getExecutionManager());


	d1.delegateOutputValueTo(d2.output);

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_TRUE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d1.getExecutionManager());

	EXPECT_TRUE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d2.getExecutionManager());
}

TEST_F(SystemTest, OutputDelegatePropagatesEmIndirect) {
	ExposedIOSystem<double> d1, d2;
	d1.delegateOutputValueTo(d2.output);

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_FALSE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d1.getExecutionManager());

	EXPECT_FALSE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d2.getExecutionManager());


	in.delegateOutputValueTo(d1.output);

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_TRUE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d1.getExecutionManager());

	EXPECT_TRUE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d2.getExecutionManager());
}

TEST_F(SystemTest, OutputUndelegatePropagatesEm) {
	ExposedIOSystem<double> d1, d2;
	in.delegateOutputValueTo(d1.output);
	d1.delegateOutputValueTo(d2.output);

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_TRUE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d1.getExecutionManager());

	EXPECT_TRUE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, d2.getExecutionManager());


	in.undelegate();

	EXPECT_TRUE(in.hasExecutionManager());
	EXPECT_TRUE(in.hasDirectExecutionManager());
	EXPECT_EQ(&mem, in.getExecutionManager());

	EXPECT_FALSE(d1.hasExecutionManager());
	EXPECT_FALSE(d1.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d1.getExecutionManager());

	EXPECT_FALSE(d2.hasExecutionManager());
	EXPECT_FALSE(d2.hasDirectExecutionManager());
	EXPECT_EQ(NULL, d2.getExecutionManager());
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

TEST_F(SystemDeathTest, InputGetValueDiesWhenNotManaged) {

	systems::connect(out.output, in.input);
	out.setOutputValue(12.3);

	in.getInputValue();  // Shouldn't die


	// Should die because in doesn't have an EM
	mem.stopManaging(in);
	ASSERT_DEATH(in.getInputValue(), "");

	mem.startManaging(out);
	ASSERT_DEATH(in.getInputValue(), "");
}

TEST_F(SystemDeathTest, OutputDelegateMixedEm) {
	systems::ManualExecutionManager localMem;
	localMem.startManaging(out);

	EXPECT_DEATH(out.delegateOutputValueTo(in.output), "");
	EXPECT_DEATH(in.delegateOutputValueTo(out.output), "");
}


}
