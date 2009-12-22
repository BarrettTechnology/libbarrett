/*
 * pid_controller.cpp
 *
 *  Created on: Nov 11, 2009
 *      Author: dc
 */


#include <cmath>
#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/constant.h>
#include <barrett/systems/pid_controller.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;

typedef units::Array<5> i_type;


class PIDControllerTest : public ::testing::Test {
public:
	PIDControllerTest() :
		pid(), feedbackSignal(i_type()), eios(), a()
	{
		systems::connect(feedbackSignal.output, pid.feedbackInput);
		systems::connect(eios.output, pid.referenceInput);
		systems::connect(pid.controlOutput, eios.input);
	}

protected:
	systems::PIDController<i_type, i_type> pid;
	systems::Constant<i_type> feedbackSignal;
	ExposedIOSystem<i_type> eios;
	i_type a;
};


TEST_F(PIDControllerTest, GainsZeroInitilized) {
	// kp
	a.assign(1);
	eios.setOutputValue(a);
	EXPECT_EQ(i_type(), eios.getInputValue());

	// ki
	a.assign(1);
	eios.setOutputValue(a);
	EXPECT_EQ(i_type(), eios.getInputValue());

	// kd
	a.assign(1e3);
	eios.setOutputValue(a);
	EXPECT_EQ(i_type(), eios.getInputValue());
}

TEST_F(PIDControllerTest, SetKp) {
	a.assign(38);
	pid.setKp(a);

	for (size_t i = 0; i < 10; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(a*a, eios.getInputValue());
	}
}

TEST_F(PIDControllerTest, SetKi) {
	a.assign(500);
	pid.setKi(a);

	a.assign(1);
	for (size_t i = 0; i < 10; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(a*i, eios.getInputValue());
	}
}

TEST_F(PIDControllerTest, SetKd) {
	a.assign(10*0.002);
	pid.setKd(a);

	size_t i = 0;
	a.assign(i);
	eios.setOutputValue(a);
	EXPECT_EQ(i_type(0.0), eios.getInputValue());
	for (i = 1; i < 10; ++i) {
		a.assign(i);
		eios.setOutputValue(a);
		EXPECT_EQ(i_type(10), eios.getInputValue());
	}
}

TEST_F(PIDControllerTest, SetIntegratorState) {
	a.assign(10);
	pid.setKi(a);

	a.assign(1);
	pid.setIntegratorState(a);

	a.assign(0);
	for (size_t i = 0; i < 10; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(i_type(1), eios.getInputValue());
	}
}

TEST_F(PIDControllerTest, SetIntegratorLimit) {
	a.assign(500);
	pid.setKi(a);

	a.assign(5.8);
	pid.setIntegratorLimit(a);

	a.assign(1);
	for (size_t i = 0; i <= 5; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(a*i, eios.getInputValue());
	}
	for (size_t i = 6; i < 10; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(i_type(5.8), eios.getInputValue());
	}

	a.assign(-1);
	for (size_t i = 0; i <= 11; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(5.8 + a*i, eios.getInputValue());
	}
	for (size_t i = 12; i < 15; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(i_type(-5.8), eios.getInputValue());
	}

	// the output isn't saturated, just the integrator
	a.assign(500);
	pid.setKp(a);

	a.assign(-1);
	eios.setOutputValue(a);
	EXPECT_EQ(i_type(-505.8), eios.getInputValue());
}

TEST_F(PIDControllerTest, SetControlSignalLimit) {
	a.assign(10);
	pid.setKp(a);
	pid.setControlSignalLimit(a);

	eios.setOutputValue(i_type(0.1));
	EXPECT_EQ(i_type(0.1) * a, eios.getInputValue());
	eios.setOutputValue(i_type(2.0));
	EXPECT_EQ(a, eios.getInputValue());
	eios.setOutputValue(i_type(-5));
	EXPECT_EQ(-a, eios.getInputValue());
}

TEST_F(PIDControllerTest, ResetIntegrator) {
	pid.setKi(i_type(500));

	a.assign(1);
	for (size_t i = 0; i < 10; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(a*i, eios.getInputValue());
	}

	pid.resetIntegrator();

	// start at i=1 because (this time through) the previous input sample
	// wasn't zero
	for (size_t i = 1; i < 10; ++i) {
		eios.setOutputValue(a);
		EXPECT_EQ(a*i, eios.getInputValue());
	}
}


}
