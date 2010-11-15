/*
 * utils.cpp
 *
 *  Created on: Nov 10, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/math/array.h>
#include <barrett/math/utils.h>


namespace {


const size_t N = 5;


class MathUtilsTest : public ::testing::Test {
protected:
	barrett::math::Array<N> a, b, e;
};


TEST_F(MathUtilsTest, SignTest) {
	a << -10, -0.5, 0, 0.5, 10;
	e <<  -1,   -1, 0,   1, 1;
	EXPECT_EQ(e, barrett::math::sign(a));
}

TEST_F(MathUtilsTest, AbsTest) {
	a << -10, -0.5, 0, 0.5, 10;
	e <<  10,  0.5, 0, 0.5, 10;
	EXPECT_EQ(e, barrett::math::abs(a));
}

TEST_F(MathUtilsTest, MinTest) {
	a << -10, -0.5,  0,  0.5, 10;
	b <<  -9,  0.5, -1, 0.25, 11;
	e << -10, -0.5, -1, 0.25, 10;
	EXPECT_EQ(e, barrett::math::min(a, b));
}

TEST_F(MathUtilsTest, MaxTest) {
	a << -10, -0.5,  0,  0.5, 10;
	b <<  -9,  0.5, -1, 0.25, 11;
	e <<  -9,  0.5,  0,  0.5, 11;
	EXPECT_EQ(e, barrett::math::max(a, b));  //NOLINT: irrelevant
}

TEST_F(MathUtilsTest, SaturateTest) {
	a << -1, -0.5, -8, 5, 10;
	b << 0.75, 0.75, 7, 7, 7;
	e << -0.75, -0.5, -7, 5, 7;
	EXPECT_EQ(e, barrett::math::saturate(a, b));
}


}