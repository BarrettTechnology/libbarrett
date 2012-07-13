/*
 * spline.cpp
 *
 *  Created on: Dec 22, 2009
 *      Author: dc
 */


#include <iostream>
#include <vector>
#include <boost/tuple/tuple.hpp>

#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/math/spline.h>


// TODO(dc): these tests could be more thorough


namespace {
using namespace barrett;

const size_t DOF = 5;
typedef units::JointPositions<DOF>::type jp_type;

TEST(SplineTest, ImplicitParameter) {
	jp_type jp;
	std::vector<jp_type> points;

	jp.setConstant(0);
	points.push_back(jp);
	jp.setConstant(1);
	points.push_back(jp);
	jp.setConstant(2);
	points.push_back(jp);

	math::Spline<jp_type> spline(points);


	EXPECT_EQ(0.0, spline.initialS());

	jp.setConstant(1.5);
	EXPECT_EQ(jp, spline.eval(spline.changeInS() * 3/4));
}

/*
TEST(SplineTest, InitialDirection) {
	jp_type jp;
	std::vector<jp_type> points;

	jp.setConstant(0);
	points.push_back(jp);
	jp.setConstant(1);
	points.push_back(jp);
	jp.setConstant(2);
	points.push_back(jp);

	units::JointVelocities<DOF>::type jv(-1.0);
	math::Spline<jp_type> spline(points, jv);


	EXPECT_EQ(0.0, spline.initialS());

	// the spline should go the "wrong" direction at first...
	jp.setConstant(0.0);
	EXPECT_TRUE((spline.eval(spline.changeInS() * 0.1).cwise() < jp).all());
}
*/

TEST(SplineTest, ExplicitParameter) {
	typedef math::Spline<jp_type>::tuple_type tuple_type;
	tuple_type sample;
	std::vector<tuple_type> samples;

	sample.get<0>() = -2.0;
	sample.get<1>().setConstant(-12.8);
	samples.push_back(sample);

	sample.get<0>() = 5.0;
	sample.get<1>().setConstant(2.0);
	samples.push_back(sample);

	math::Spline<jp_type> spline(samples);


	EXPECT_EQ(-2.0, spline.initialS());
	EXPECT_EQ(5.0, spline.finalS());
	EXPECT_EQ(7.0, spline.changeInS());

	jp_type jp;

	jp = spline.eval(-2.0);
	for (size_t i = 0; i < DOF; ++i) {
		EXPECT_DOUBLE_EQ(-12.8, jp[i]);
	}

	jp = spline.eval(5.0);
	for (size_t i = 0; i < DOF; ++i) {
		EXPECT_DOUBLE_EQ(2, jp[i]);
	}
}

TEST(SplineTest, SaturateByDefault) {
	typedef math::Spline<jp_type>::tuple_type tuple_type;
	tuple_type sample;
	std::vector<tuple_type> samples;

	sample.get<0>() = -2.0;
	sample.get<1>().setConstant(-12.8);
	samples.push_back(sample);

	sample.get<0>() = 5.0;
	sample.get<1>().setConstant(2.0);
	samples.push_back(sample);

	math::Spline<jp_type> spline(samples);


	EXPECT_EQ(-2.0, spline.initialS());
	EXPECT_EQ(5.0, spline.finalS());
	EXPECT_EQ(7.0, spline.changeInS());

	jp_type jp;

	jp = spline.eval(-4.0);
	for (size_t i = 0; i < DOF; ++i) {
		EXPECT_DOUBLE_EQ(-12.8, jp[i]);
	}

	jp = spline.eval(10.0);
	for (size_t i = 0; i < DOF; ++i) {
		EXPECT_DOUBLE_EQ(2, jp[i]);
	}
}

TEST(SplineTest, NoSaturateCtor) {
	typedef math::Spline<jp_type>::tuple_type tuple_type;
	tuple_type sample;
	std::vector<tuple_type> samples;

	sample.get<0>() = -2.0;
	sample.get<1>().setConstant(-12.8);
	samples.push_back(sample);

	sample.get<0>() = 5.0;
	sample.get<1>().setConstant(2.0);
	samples.push_back(sample);

	math::Spline<jp_type> spline(samples, false);


	EXPECT_EQ(-2.0, spline.initialS());
	EXPECT_EQ(5.0, spline.finalS());
	EXPECT_EQ(7.0, spline.changeInS());

	jp_type jp;

	jp = spline.eval(-4.0);
	for (size_t i = 0; i < DOF; ++i) {
		EXPECT_NE(-12.8, jp[i]);
	}

	jp = spline.eval(10.0);
	for (size_t i = 0; i < DOF; ++i) {
		EXPECT_NE(2, jp[i]);
	}
}


}
