/*
 * rate_limiter.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems/rate_limiter.h>

#include <barrett/math/utils.h>
#include <barrett/math/matrix.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/helpers.h>
#include "exposed_io_system.h"


namespace {
using namespace barrett;


template<typename T>
void expectEqual(const T& a, const T& b) {
	EXPECT_TRUE(math::abs(a - b).maxCoeff() < 1e-9) << "a = " << a << "; b = " << b;
}

template<>
void expectEqual(const double& a, const double& b) {
	EXPECT_DOUBLE_EQ(a, b);
}

template<typename T>
class RateLimiterTest : public ::testing::Test {
public:
	RateLimiterTest() : mem(T_s) {
		mem.startManaging(eios);
	}

	void startTesting(systems::RateLimiter<T>& rl) {
		systems::connect(eios.output, rl.input);
		systems::connect(rl.output, eios.input);
	}

	void setAndRun(const T& inputValue) {
		eios.setOutputValue(inputValue);
		mem.runExecutionCycle();
	}

	void setRunAndExpect(const T& inputValue, const T& expectedOutputValue) {
		setAndRun(inputValue);
		expectEqual(expectedOutputValue, eios.getInputValue());
	}

protected:
	static const double T_s = 0.1;
	systems::ManualExecutionManager mem;
	ExposedIOSystem<T> eios;
};


// Test for T = double
typedef RateLimiterTest<double> RateLimiterDoubleTest;

TEST_F(RateLimiterDoubleTest, DefaultCtorDoesNotLimit) {
	systems::RateLimiter<double> rl;
	startTesting(rl);

	setRunAndExpect(9e9, 9e9);
	setRunAndExpect(-9e9, -9e9);
	setRunAndExpect(0.0, 0.0);
}

TEST_F(RateLimiterDoubleTest, InitialValueIsZero) {
	double rate = 10.0;
	systems::RateLimiter<double> rl(rate);
	startTesting(rl);

	setRunAndExpect(9e9, rate * T_s);
}

TEST_F(RateLimiterDoubleTest, LimitCanBeModified) {
	double rate1 = 10.0;
	systems::RateLimiter<double> rl(rate1);
	startTesting(rl);

	double value = 0.5 * rate1*T_s;
	setRunAndExpect(0.5, value);

	value += rate1*T_s;
	setRunAndExpect(value + 0.001, value);

	double rate2 = 100.0;
	rl.setLimit(rate2);

	value += rate2*T_s;
	setRunAndExpect(value + 0.001, value);

	value += 0.5 * rate2*T_s;
	setRunAndExpect(value, value);
}

TEST_F(RateLimiterDoubleTest, DoesntLimitSlowChanges) {
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

TEST_F(RateLimiterDoubleTest, RampsUp) {
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

TEST_F(RateLimiterDoubleTest, RampsDown) {
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

TEST_F(RateLimiterDoubleTest, SwitchesDirection) {
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

TEST_F(RateLimiterDoubleTest, GetsSamplePeriodFromEM) {
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


// Test for T = v_type
typedef math::Vector<3>::type v_type;
typedef RateLimiterTest<v_type> RateLimiterVectorTest;


const v_type ZERO_VECTOR(0.0);
const v_type BIG_VECTOR(9e9);

TEST_F(RateLimiterVectorTest, DefaultCtorDoesNotLimit) {
	systems::RateLimiter<v_type> rl;
	startTesting(rl);

	setRunAndExpect(BIG_VECTOR, BIG_VECTOR);
	setRunAndExpect(-BIG_VECTOR, -BIG_VECTOR);
	setRunAndExpect(ZERO_VECTOR, ZERO_VECTOR);
}

TEST_F(RateLimiterVectorTest, InitialValueIsZero) {
	v_type rate(10.0, 5.0, 1.0);
	systems::RateLimiter<v_type> rl(rate);
	startTesting(rl);

	setRunAndExpect(BIG_VECTOR, rate * T_s);
}

TEST_F(RateLimiterVectorTest, RampsUp) {
	v_type input(1.0);
	v_type rate(1.0);
	systems::RateLimiter<v_type> rl(rate);
	startTesting(rl);

	for (int i = 0; i < 10; ++i) {
		setRunAndExpect(input, (1+i) * T_s*rate);
	}
	for (int i = 0; i < 10; ++i) {
		setRunAndExpect(input, input);
	}
}


// Death tests
TEST(RateLimiterDoubleDeathTest, LimitCantBeNegative) {
	systems::RateLimiter<double> rl;
	EXPECT_DEATH(rl.setLimit(-0.00001), "");

	EXPECT_DEATH(systems::RateLimiter<double> rl2(-1.0), "");
}

TEST(RateLimiterVectorDeathTest, LimitCantBeNegative) {
	systems::RateLimiter<v_type> rl;
	EXPECT_DEATH(rl.setLimit(v_type(1.0, 2.0, -0.0001)), "");

	EXPECT_DEATH(systems::RateLimiter<v_type> rl2(v_type(-1.0)), "");
}


}
