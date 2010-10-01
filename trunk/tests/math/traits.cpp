/*
 * traits.cpp
 *
 *  Created on: Mar 24, 2010
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/math/traits.h>


namespace {
using namespace barrett;


TEST(MathTraitsTest, Double) {
	typedef math::Traits<double> T;

	double t = 7;
	T::zero(t);
	EXPECT_EQ(0.0, t);
	EXPECT_EQ(0.0, T::zero());

	EXPECT_EQ(8.0, T::add(2, 6));
	EXPECT_EQ(-4.0, T::sub(2, 6));
	EXPECT_EQ(-2.0, T::neg(2));
	EXPECT_EQ(8.0, T::mult(2, 4));
	EXPECT_EQ(8.0, T::div(16, 2));
}


}
