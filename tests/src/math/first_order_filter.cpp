/*
 * first_order_filter.cpp
 *
 *  Created on: Apr 1, 2010
 *      Author: dc
 */

#include <math.h>

#include <gtest/gtest.h>
#include <barrett/math/matrix.h>
#include <barrett/math/first_order_filter.h>


namespace {
using namespace barrett;


const double T_s = 0.1;
const double ERR = 1e-5;


// TODO(dc): finish testing this!
class FirstOrderFilterTest : public ::testing::Test {
public:
	FirstOrderFilterTest() :
		f(T_s) {}

protected:
	math::FirstOrderFilter<double> f;
};


TEST_F(FirstOrderFilterTest, DefaultCtor) {
	math::FirstOrderFilter<double> f;

	f.setIntegrator();
	EXPECT_EQ(0.0, f(1.0));
}

TEST_F(FirstOrderFilterTest, TimeStepCtor) {
	f.setIntegrator();
	EXPECT_EQ(T_s, f(1.0));
}

TEST_F(FirstOrderFilterTest, SetSamplePeriod) {
	int i;
	f.setIntegrator();

	f.setSamplePeriod(1.0);
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(i, f(1.0), ERR);
	}

	f.setSamplePeriod(10.0);
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(10 + i*10, f(1.0), ERR);
	}

	f.setSamplePeriod(0.002);
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(110 + i*0.002, f(1.0), ERR);
	}
}

TEST_F(FirstOrderFilterTest, SetLowPass) {
	int i;

	f.setLowPass(0.001, 0.25);
	for (i = 1; i <= 300; ++i) {
		ASSERT_NEAR(0.25*(1.0 - exp(-i*T_s * 0.001)), f(1.0), ERR);
	}
	for (i = 1; i <= 300; ++i) {
		ASSERT_NEAR(0.25*(1.0 - exp(-(i+300)*T_s * 0.001) - 16.0*(1.0 - exp(-i*T_s * 0.001))), f(-15.0), ERR);
	}
	for (i = 1; i <= 300; ++i) {
		ASSERT_NEAR(0.25*(1.0 - exp(-(i+600)*T_s * 0.001) - 16.0*(1.0 - exp(-(i+300)*T_s * 0.001)) + 15.0*(1.0 - exp(-i*T_s * 0.001))), f(0.0), ERR);
	}
}

TEST_F(FirstOrderFilterTest, SetHighPass) {
	int i;

	f.setHighPass(0.001, 0.5);
	for (i = 1; i <= 300; ++i) {
		ASSERT_NEAR(0.5*exp(-i*T_s * 0.001), f(1.0), ERR);
	}
	for (i = 1; i <= 300; ++i) {
		ASSERT_NEAR(0.5*(exp(-(i+300)*T_s * 0.001) + 2.6 * exp(-i*T_s * 0.001)), f(3.6), ERR);
	}
	for (i = 1; i <= 300; ++i) {
		ASSERT_NEAR(0.5*(exp(-(i+600)*T_s * 0.001) + 2.6 * exp(-(i+300)*T_s * 0.001) - 13.6 * exp(-i*T_s * 0.001)), f(-10.0), ERR);
	}
}

TEST_F(FirstOrderFilterTest, SetIntegrator) {
	int i;

	f.setIntegrator(1.0);
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(i*T_s, f(1.0), ERR);
	}

	f.setIntegrator(0.5);
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(10*T_s + i*0.5*T_s, f(1.0), ERR);
	}

	f.setIntegrator(-200.0);
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(15*T_s - i*T_s*200.0, f(1.0), ERR);
	}
	for (i = 1; i <= 10; ++i) {
		ASSERT_NEAR(-1985*T_s + i*T_s*200.0, f(-1.0), ERR);
	}
}


TEST_F(FirstOrderFilterTest, Matrix) {
	typedef math::Vector<3>::type vector_t;
	math::FirstOrderFilter<math::Vector<3>::type> f(T_s);
	int i;

	vector_t omega(0.001, 0.0003, 0.0005);
	vector_t gain(0.1, 0.2, 0.3);
	vector_t expected, actual;

	f.setLowPass(omega, gain);
	for (i = 1; i <= 300; ++i) {
		expected = gain.cwise()*(1.0 + (-(-i*T_s * omega).cwise().exp()).cwise());
		actual = f(vector_t(1.0));
		ASSERT_LT((expected - actual).cwise().abs().maxCoeff(), ERR);
	}
	for (i = 1; i <= 300; ++i) {
		expected = gain.cwise()  *  (
				1.0 * (1.0 + (-(-(i+300)*T_s * omega).cwise().exp()).cwise()) +
				-16.0 * (1.0 + (-(-i*T_s * omega).cwise().exp()).cwise())
				);
		actual = f(vector_t(-15.0));
		ASSERT_LT((expected - actual).cwise().abs().maxCoeff(), ERR);
	}
	for (i = 1; i <= 300; ++i) {
		expected = gain.cwise()  *  (
				1.0 * (1.0 + (-(-(i+600)*T_s * omega).cwise().exp()).cwise()) +
				-16.0 * (1.0 + (-(-(i+300)*T_s * omega).cwise().exp()).cwise()) +
				15.0 * (1.0 + (-(-i*T_s * omega).cwise().exp()).cwise())
				);
		actual = f(vector_t(0.0));
		ASSERT_LT((expected - actual).cwise().abs().maxCoeff(), ERR);
	}
}


}
