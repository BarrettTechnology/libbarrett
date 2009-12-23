/*
 * spline.cpp
 *
 *  Created on: Dec 22, 2009
 *      Author: dc
 */


#include <iostream>
#include <vector>
#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/math/spline.h>


// TODO(dc): these tests could be more thorough


namespace {
using namespace barrett;
const size_t DOF = 5;


TEST(SplineTest, ImplicitParameter) {
	units::JointPositions<DOF> jp;
	std::vector<units::JointPositions<DOF> > points;

	jp.assign(0);
	points.push_back(jp);
	jp.assign(1);
	points.push_back(jp);
	jp.assign(2);
	points.push_back(jp);

	math::Spline<units::JointPositions<DOF> > spline(points);


	EXPECT_EQ(0.0, spline.initialX());

	jp.assign(1.5);
	EXPECT_EQ(jp, spline.eval(spline.changeInX() * 3/4));
}

TEST(SplineTest, ExplicitParameter) {
	math::Spline<units::JointPositions<DOF> >::Sample sample;
	std::vector<math::Spline<units::JointPositions<DOF> >::Sample > samples;

	sample.x = -2.0;
	sample.point.assign(-12.8);
	samples.push_back(sample);

	sample.x = 5.0;
	sample.point.assign(2.0);
	samples.push_back(sample);

	math::Spline<units::JointPositions<DOF> > spline(samples);


	EXPECT_EQ(-2.0, spline.initialX());
	EXPECT_EQ(5.0, spline.finalX());
	EXPECT_EQ(7.0, spline.changeInX());

	units::JointPositions<DOF> jp;

	jp = spline.eval(-2.0);
	for (size_t i = 0; i < jp.size(); ++i) {
		EXPECT_DOUBLE_EQ(-12.8, jp[i]);
	}

	jp = spline.eval(5.0);
	for (size_t i = 0; i < jp.size(); ++i) {
		EXPECT_DOUBLE_EQ(2, jp[i]);
	}
}


}
