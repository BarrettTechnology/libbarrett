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
		systems::connect(eios.output, rl.input);
		systems::connect(rl.output, eios.input);
	}

	void setAndRun(double inputValue) {
		eios.setOutputValue(inputValue);
		mem.runExecutionCycle();
	}

	void setRunAndExpect(double inputValue, double expectedOutputValue) {
		setAndRun(inputValue);
		EXPECT_EQ(expectedOutputValue, eios.getInputValue());
	}

protected:
	static const double T_s = 0.1;
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> eios;
	systems::RateLimiter<double> rl;
};


TEST_F(RateLimiterTest, DefaultCtorDoesNotLimit) {
	setRunAndExpect(0.0, 0.0);
	setRunAndExpect(9e9, 9e9);
	setRunAndExpect(-9e9, -9e9);
}


}
