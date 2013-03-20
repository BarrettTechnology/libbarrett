/*
 * utils.cpp
 *
 *  Created on: Nov 10, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/math/matrix.h>
#include <barrett/math/utils.h>


namespace {
using namespace barrett;


const size_t N = 5;


class MathUtilsTest : public ::testing::Test {
protected:
	math::Vector<N>::type a, b, e;
};


TEST_F(MathUtilsTest, VectorSignTest) {
	a << -10, -0.5, 0, 0.5, 10;
	e <<  -1,   -1, 0,   1, 1;
	EXPECT_EQ(e, math::sign(a));
}

TEST_F(MathUtilsTest, ScalarSignTest) {
	EXPECT_EQ(1.0, math::sign(10.3));
	EXPECT_EQ(0.0, math::sign(0));
	EXPECT_EQ(-1.0, math::sign(-5));
}

TEST_F(MathUtilsTest, VectorAbsTest) {
	a << -10, -0.5, 0, 0.5, 10;
	e <<  10,  0.5, 0, 0.5, 10;
	EXPECT_EQ(e, math::abs(a));
}

TEST_F(MathUtilsTest, ScalarAbsTest) {
	EXPECT_EQ(10.3, math::abs(10.3));
	EXPECT_EQ(0.0, math::abs(0));
	EXPECT_EQ(5, math::abs(-5));
}

TEST_F(MathUtilsTest, VectorMinTest) {
	a << -10, -0.5,  0,  0.5, 10;
	b <<  -9,  0.5, -1, 0.25, 11;
	e << -10, -0.5, -1, 0.25, 10;
	EXPECT_EQ(e, math::min(a, b));
}

TEST_F(MathUtilsTest, ScalarMinTest) {
	EXPECT_EQ(5, math::min(10.3, 5));
	EXPECT_EQ(5, math::min(5, 10.3));
	EXPECT_EQ(-5, math::min(10.3, -5));
	EXPECT_EQ(-5, math::min(-5, 10.3));
	EXPECT_EQ(-10.3, math::min(-10.3, -5));
	EXPECT_EQ(-10.3, math::min(-5, -10.3));
}

TEST_F(MathUtilsTest, VectorMaxTest) {
	a << -10, -0.5,  0,  0.5, 10;
	b <<  -9,  0.5, -1, 0.25, 11;
	e <<  -9,  0.5,  0,  0.5, 11;
	EXPECT_EQ(e, math::max(a, b));  //NOLINT: irrelevant
}

TEST_F(MathUtilsTest, ScalarMaxTest) {
	EXPECT_EQ(-5, math::max(-10.3, -5));
	EXPECT_EQ(-5, math::max(-5, -10.3));
	EXPECT_EQ(5, math::max(-10.3, 5));
	EXPECT_EQ(5, math::max(5, -10.3));
	EXPECT_EQ(10.3, math::max(10.3, 5));
	EXPECT_EQ(10.3, math::max(5, 10.3));
}

TEST_F(MathUtilsTest, VectorVectorSaturateTest1) {
	a << -1, -0.5, -8, 5, 10;
	b << 0.75, 0.75, 7, 7, 7;
	e << -0.75, -0.5, -7, 5, 7;
	EXPECT_EQ(e, math::saturate(a, b));
}

TEST_F(MathUtilsTest, VectorScalarSaturateTest1) {
	a << -1, -0.5, -8, 5, 10;
	e << -1, -0.5, -6, 5, 6;
	EXPECT_EQ(e, math::saturate(a, 6));
}

TEST_F(MathUtilsTest, ScalarScalarSaturateTest1) {
	EXPECT_EQ(5, math::saturate(10.3, 5.0));
	EXPECT_EQ(1.6, math::saturate(1.6, 5.0));
	EXPECT_EQ(-1.6, math::saturate(-1.6, 5.0));
	EXPECT_EQ(-5, math::saturate(-12342.0, 5.0));
}

TEST_F(MathUtilsTest, VectorScalarSaturateTest2) {
	a << -1, -0.5, -8, 5, 10;
	e << 0.0, 0.0, 0.0, 5, 6;
	EXPECT_EQ(e, math::saturate(a, 0, 6));
}

TEST_F(MathUtilsTest, ScalarScalarSaturateTest2) {
	EXPECT_EQ(5, math::saturate(10.3, -1.0, 5.0));
	EXPECT_EQ(1.6, math::saturate(1.6, -1.0, 5.0));
	EXPECT_EQ(-1.0, math::saturate(-1.6, -1.0, 5.0));
	EXPECT_EQ(-1.0, math::saturate(-12342.0, -1.0, 5.0));
}

TEST_F(MathUtilsTest, VectorVectorDeadbandTest) {
	a << -1, -0.5, -8, 5, 10;
	b << 0.75, 0.75, 7, 7, 7;
	e << -0.25, 0, -1, 0, 3;
	EXPECT_EQ(e, math::deadband(a, b));
}

TEST_F(MathUtilsTest, VectorScalarDeadbandTest) {
	a << -1, -0.5, -8, 5, 10;
	e << 0, 0, -2, 0, 4;
	EXPECT_EQ(e, math::deadband(a, 6));
}

TEST_F(MathUtilsTest, ScalarScalarDeadbandTest) {
	EXPECT_DOUBLE_EQ(5.3, math::deadband(10.3, 5.0));
	EXPECT_DOUBLE_EQ(0, math::deadband(1.6, 5.0));
	EXPECT_DOUBLE_EQ(0, math::deadband(-1.6, 5.0));
	EXPECT_DOUBLE_EQ(-12337, math::deadband(-12342.0, 5.0));
}


}
