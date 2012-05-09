/*
 * first_order_filter.cpp
 *
 *  Created on: Apr 1, 2010
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/math/matrix.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/first_order_filter.h>

#include "exposed_io_system.h"


namespace {
using namespace barrett;


const double T_s = 0.1;


TEST(SystemsFirstOrderFilterTest, GetPeriodFromEm) {
	systems::ManualExecutionManager mem(T_s);
	ExposedIOSystem<double> eios;
	systems::FirstOrderFilter<double> f;

	mem.startManaging(eios);
	f.setIntegrator();

	systems::connect(f.output, eios.input);
	systems::connect(eios.output, f.input);

	eios.setOutputValue(10);
	mem.runExecutionCycle();
	EXPECT_DOUBLE_EQ(1*10.0*T_s, eios.getInputValue());
	mem.runExecutionCycle();
	EXPECT_DOUBLE_EQ(2*10.0*T_s, eios.getInputValue());
	mem.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s, eios.getInputValue());
}

TEST(SystemsFirstOrderFilterTest, SwitchEm) {
	systems::ManualExecutionManager mem1(T_s);
	ExposedIOSystem<double> in, out;
	systems::FirstOrderFilter<double> f;

	mem1.startManaging(in);
	f.setIntegrator();

	systems::connect(f.output, in.input);
	systems::connect(out.output, f.input);

	out.setOutputValue(10);
	mem1.runExecutionCycle();
	EXPECT_DOUBLE_EQ(1*10.0*T_s, in.getInputValue());
	mem1.runExecutionCycle();
	EXPECT_DOUBLE_EQ(2*10.0*T_s, in.getInputValue());
	mem1.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s, in.getInputValue());


	systems::ManualExecutionManager mem2(1.0);
	mem2.startManaging(in);

	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 1*10.0, in.getInputValue());
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 2*10.0, in.getInputValue());
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 3*10.0, in.getInputValue());
}


// death tests
TEST(SystemsFirstOrderFilterDeathTest, InvalidEmPeriod) {
	systems::ManualExecutionManager mem(0.0);
	ExposedIOSystem<double> eios;
	systems::FirstOrderFilter<double> f;

	mem.startManaging(eios);
	f.setIntegrator();

	EXPECT_DEATH(systems::connect(f.output, eios.input), "");
}


}
