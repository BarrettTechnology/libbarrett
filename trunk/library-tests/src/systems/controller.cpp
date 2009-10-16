/*
 * controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;


// TODO(dc): actually test this
class ControllerTest : public ::testing::Test {
protected:
	systems::Controller<double> controller;
};


TEST_F(ControllerTest, InitialOutputValueCtor) {
	systems::Controller<double> ivController(-88.1);
	ExposedIOSystem<double> eios;
	systems::connect(ivController.controlOutput, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "initial value undefined";
	EXPECT_EQ(-88.1, eios.getInputValue())
		<< "wrong initial value given";
}

TEST_F(ControllerTest, GetReferenceInput) {
	EXPECT_EQ(&controller.referenceInput, controller.getReferenceInput())
		<< "getReferenceInput() didn't return &controller.referenceInput";
}

TEST_F(ControllerTest, GetFeedbackInput) {
	EXPECT_EQ(&controller.feedbackInput, controller.getFeedbackInput())
		<< "getFeedbackInput() didn't return &controller.feedbackInput";
}

TEST_F(ControllerTest, GetControlOutput) {
	EXPECT_EQ(&controller.controlOutput, controller.getControlOutput())
		<< "getControlOutput() didn't return &controller.controlOutput";
}

TEST_F(ControllerTest, SelectAdapter) {
	// TODO(dc): stub
}


}
