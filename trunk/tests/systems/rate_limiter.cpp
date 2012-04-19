/*
 * rate_limiter.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems/rate_limiter.h>

#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/helpers.h>
#include "exposed_io_system.h"


namespace {
using namespace barrett;

class RateLimiterTest : public ::testing::Test {
public:
	RateLimiterTest() : mem(T_s) {
		mem.startManaging(eios);
	}

	void startTesting(systems::RateLimiter<double>& rl) {
		systems::connect(eios.output, rl.input);
		systems::connect(rl.output, eios.input);
	}

	void setAndRun(double inputValue) {
		eios.setOutputValue(inputValue);
		mem.runExecutionCycle();
	}

	void setRunAndExpect(double inputValue, double expectedOutputValue) {
		setAndRun(inputValue);
		EXPECT_DOUBLE_EQ(expectedOutputValue, eios.getInputValue());
	}

protected:
	static const double T_s = 0.1;
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> eios;
};


TEST_F(RateLimiterTest, DefaultCtorDoesNotLimit) {
	systems::RateLimiter<double> rl;
	startTesting(rl);

	setRunAndExpect(9e9, 9e9);
	setRunAndExpect(-9e9, -9e9);
	setRunAndExpect(0.0, 0.0);
}

TEST_F(RateLimiterTest, InitialValueIsZero) {
	double rate = 10.0;
	systems::RateLimiter<double> rl(rate);
	startTesting(rl);

	setRunAndExpect(9e9, rate * T_s);
}

TEST_F(RateLimiterTest, DoesntLimitSlowChanges) {
	systems::RateLimiter<double> rl(3.5);
	startTesting(rl);

	setRunAndExpect(0.3, 0.3);
	setRunAndExpect(0.5, 0.5);
	setRunAndExpect(0.83, 0.83);
	setRunAndExpect(0.8, 0.8);
	setRunAndExpect(0.46, 0.46);
	setRunAndExpect(0.11, 0.11);
	setRunAndExpect(-0.2, -0.2);
}

TEST_F(RateLimiterTest, RampsUp) {
	double rate = 1.0;
	systems::RateLimiter<double> rl(rate);
	startTesting(rl);

	for (int i = 0; i < 10; ++i) {
		setRunAndExpect(1.0, (1+i) * T_s*rate);
	}
	for (int i = 0; i < 10; ++i) {
		setRunAndExpect(1.0, 1.0);
	}
}

TEST_F(RateLimiterTest, RampsDown) {
	double rate = 3.8;
	systems::RateLimiter<double> rl(rate);
	startTesting(rl);

	for (int i = 0; i < 20; ++i) {
		setRunAndExpect(-rate*2, -(1+i) * T_s*rate);
	}
	for (int i = 0; i < 10; ++i) {
		setRunAndExpect(-rate*2, -rate*2);
	}
}

TEST_F(RateLimiterTest, SwitchesDirection) {
	double rate = 1.0;
	systems::RateLimiter<double> rl(rate);
	startTesting(rl);

	for (int i = 0; i < 8; ++i) {
		setRunAndExpect(9e9, (1+i) * T_s*rate);
	}
	for (int i = 0; i < 5; ++i) {
		setRunAndExpect(-9e9, (7-i) * T_s*rate);
	}
	for (int i = 0; i < 8; ++i) {
		setRunAndExpect(9e9, (4+i) * T_s*rate);
	}
}

TEST_F(RateLimiterTest, GetsSamplePeriodFromEM) {
	double rate = 1.0;
	systems::RateLimiter<double> rl(rate);
	startTesting(rl);

	setRunAndExpect(9e9, rate * T_s);
	setRunAndExpect(9e9, rate * 2*T_s);

	double T_s2 = 1.0;
	systems::ManualExecutionManager mem2(T_s2);
	ExposedIOSystem<double> eios2;
	mem2.startManaging(eios2);
	systems::disconnect(rl.input);
	systems::disconnect(rl.output);
	systems::connect(eios2.output, rl.input);
	systems::connect(rl.output, eios2.input);

	eios2.setOutputValue(9e9);
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(rate * (2*T_s + T_s2), eios2.getInputValue());

	eios2.setOutputValue(9e9);
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(rate * (2*T_s + 2*T_s2), eios2.getInputValue());
}


// Death tests
TEST(RateLimiterDeathTest, LimitCantBeNegative) {
	systems::RateLimiter<double> rl;
	EXPECT_DEATH(rl.setLimit(-0.00001), "");

	EXPECT_DEATH(systems::RateLimiter<double> rl2(-1.0), "");
}


}
