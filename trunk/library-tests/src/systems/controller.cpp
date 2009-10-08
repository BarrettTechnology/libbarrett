/*
 * controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems.h>


namespace {

// TODO(dc): actually test this
class ControllerTest : public ::testing::Test {
protected:
	Systems::Controller<double> controller;
};


TEST_F(ControllerTest, GetReferenceInput) {
	EXPECT_EQ(&controller.referenceInput, controller.getReferenceInput())
		<< "getReferenceInput() didn't return &controller.referenceInput";
}

TEST_F(ControllerTest, GetFeedbackInput) {
	EXPECT_EQ(&controller.feedbackInput, controller.getFeedbackInput())
		<< "getFeedbackInput() didn't return &controller.feedbackInput";
}


}
