/*
 * spline_evaluator.cpp
 *
 *  Created on: Dec 22, 2009
 *      Author: dc
 */


#include <vector>
#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/math/spline.h>
#include <barrett/systems/spline_evaluator.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;


TEST(SplineEvaluatorTest, Test) {
	ExposedIOSystem<double> xSys;
	ExposedIOSystem<units::JointTorques<10> > valueSys;

	std::vector<units::JointTorques<10> > vec(2);
	vec[1].assign(10.0);

	math::Spline<units::JointTorques<10> > s(vec);
	systems::SplineEvaluator<units::JointTorques<10> > seSys(s);

	systems::connect(xSys.output, seSys.input);
	systems::connect(seSys.output, valueSys.input);

	xSys.setOutputValue(s.changeInX() / 2.0);
	EXPECT_EQ(units::JointTorques<10>(5.0), valueSys.getInputValue());
}


}
