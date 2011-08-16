/*
 * ramp.cpp
 *
 *  Created on: May 18, 2011
 *      Author: dc
 */

#include <gtest/gtest.h>

#include <barrett/systems/ramp.h>

#include "exposed_io_system.h"


namespace {
using namespace barrett;


const double T_s = 0.001;

class RampTest : public ::testing::Test {
public:
	RampTest() :
		mem(T_s), ramp(&mem)
	{
		mem.startManaging(eios);
		systems::connect(ramp.output, eios.input);
	}

protected:
	systems::ManualExecutionManager mem;
	systems::Ramp ramp;
	ExposedIOSystem<double> eios;
};


TEST_F(RampTest, DefaultToStopped) {
	EXPECT_FALSE(ramp.isRunning());

	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_EQ(0.0, eios.getInputValue());
	}
}

TEST_F(RampTest, DefaultSlopeIsOne) {
	ramp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ((i+1)*T_s, eios.getInputValue());
	}
}

TEST_F(RampTest, SlopeCtor) {
	systems::Ramp localRamp(&mem, -3.0);
	systems::reconnect(localRamp.output, eios.input);

	EXPECT_FALSE(localRamp.isRunning());

	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_EQ(0.0, eios.getInputValue());
	}

	localRamp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(-3.0*(i+1)*T_s, eios.getInputValue());
	}
}

TEST_F(RampTest, StartStopIsRunning) {
	EXPECT_FALSE(ramp.isRunning());
	ramp.start();
	EXPECT_TRUE(ramp.isRunning());
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ((i+1)*T_s, eios.getInputValue());
	}

	EXPECT_TRUE(ramp.isRunning());
	ramp.stop();
	EXPECT_FALSE(ramp.isRunning());
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(10*T_s, eios.getInputValue());
	}

	EXPECT_FALSE(ramp.isRunning());
	ramp.start();
	EXPECT_TRUE(ramp.isRunning());
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ((i+11)*T_s, eios.getInputValue());
	}

	EXPECT_TRUE(ramp.isRunning());
	ramp.stop();
	EXPECT_FALSE(ramp.isRunning());
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(20*T_s, eios.getInputValue());
	}

	EXPECT_FALSE(ramp.isRunning());
	ramp.start();
	EXPECT_TRUE(ramp.isRunning());
	ramp.stop();
	EXPECT_FALSE(ramp.isRunning());
	ramp.start();
	EXPECT_TRUE(ramp.isRunning());
}

TEST_F(RampTest, SetSlope) {
	ramp.setSlope(-1.0);
	ramp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(-(i+1)*T_s, eios.getInputValue());
	}

	ramp.setSlope(3.0);
	ramp.stop();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(-10*T_s, eios.getInputValue());
	}

	ramp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();

		// The floating-point errors get "large" as the output crosses zero.
		EXPECT_NEAR((3*(i+1)-10)*T_s, eios.getInputValue(), 1e-10);
	}

	ramp.setSlope(10.0);
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ((20+10*(i+1))*T_s, eios.getInputValue());
	}
}

TEST_F(RampTest, SetOutput) {
	ramp.setOutput(10.0);
	ramp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(10.0 + (i+1)*T_s, eios.getInputValue());
	}

	ramp.setOutput(-8.0);
	ramp.stop();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(-8.0, eios.getInputValue());
	}

	ramp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(-8.0 + (i+1)*T_s, eios.getInputValue());
	}

	ramp.setOutput(1e3);
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(1e3 + (i+1)*T_s, eios.getInputValue());
	}

	ramp.setSlope(-1.0);
	ramp.setOutput(1.0);
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
		EXPECT_DOUBLE_EQ(1.0 - (i+1)*T_s, eios.getInputValue());
	}
}

TEST_F(RampTest, Reset) {
	ramp.start();
	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
	}

	EXPECT_DOUBLE_EQ(10*T_s, eios.getInputValue());
	ramp.reset();
	EXPECT_DOUBLE_EQ(0.0, eios.getInputValue());

	for (int i = 0; i < 10; ++i) {
		mem.runExecutionCycle();
	}
	EXPECT_DOUBLE_EQ(10*T_s, eios.getInputValue());
}

TEST_F(RampTest, SmoothStartStopIsRunning) {
	double y = 0.0, y_1 = 0.0, dy = 0.0, dy_1 = 0.0;

	EXPECT_FALSE(ramp.isRunning());
	ramp.smoothStart(10*T_s);
	EXPECT_TRUE(ramp.isRunning());

	for (int i = 0; i < 100; ++i) {
		mem.runExecutionCycle();

		y = eios.getInputValue();
		dy = y - y_1;

		EXPECT_TRUE(ramp.isRunning());
		if (i < 9) {
			EXPECT_LT(dy_1, dy);
			EXPECT_LT(dy, T_s);
		} else {
			EXPECT_NEAR(T_s, dy, 1e-10);
		}

		y_1 = y;
		dy_1 = dy;
	}

	EXPECT_TRUE(ramp.isRunning());
	ramp.smoothStop(80.5*T_s);
	EXPECT_TRUE(ramp.isRunning());

	for (int i = 0; i < 100; ++i) {
		mem.runExecutionCycle();

		y = eios.getInputValue();
		dy = y - y_1;

		if (i < 80) {
			EXPECT_TRUE(ramp.isRunning());
			EXPECT_GT(dy_1, dy);
			EXPECT_GT(dy, 0.0);
		} else {
			EXPECT_FALSE(ramp.isRunning());
			EXPECT_EQ(0.0, dy);
		}

		y_1 = y;
		dy_1 = dy;
	}
}

TEST_F(RampTest, SmoothSetSlope) {
	double y = 0.0, y_1 = 0.0, dy = 0.0, dy_1 = T_s;

	ramp.start();

	ramp.smoothSetSlope(-1.0, 30*T_s);
	for (int i = 0; i < 100; ++i) {
		mem.runExecutionCycle();

		y = eios.getInputValue();
		dy = y - y_1;

		if (i < 29) {
			EXPECT_GT(dy_1, dy);
			EXPECT_GT(dy, -T_s);
		} else {
			EXPECT_NEAR(-T_s, dy, 1e-10);
		}

		y_1 = y;
		dy_1 = dy;
	}

	ramp.smoothSetSlope(10.0, 30.6453*T_s);
	for (int i = 0; i < 100; ++i) {
		mem.runExecutionCycle();

		y = eios.getInputValue();
		dy = y - y_1;

		if (i < 30) {
			EXPECT_LT(dy_1, dy);
			EXPECT_LT(dy, 10.0*T_s);
		} else {
			EXPECT_NEAR(10.0*T_s, dy, 1e-10);
		}

		y_1 = y;
		dy_1 = dy;
	}
}

TEST_F(RampTest, MixedSetSlope) {
	double y = 0.0, y_1 = 0.0;

	ramp.start();
	ramp.smoothSetSlope(-1.0, 30*T_s);
	for (int i = 0; i < 15; ++i) {
		mem.runExecutionCycle();
	}
	y_1 = eios.getInputValue();

	ramp.setSlope(3.0);
	for (int i = 0; i < 100; ++i) {
		mem.runExecutionCycle();

		y = eios.getInputValue();
		EXPECT_NEAR(3.0*T_s, y - y_1, 1e-10);
		y_1 = y;
	}

}


}
