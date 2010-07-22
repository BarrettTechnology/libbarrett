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


TEST(SystemsFirstOrderFilterTest, NoEMOnConstruction) {
	ExposedIOSystem<double> eios;
	systems::FirstOrderFilter<double> f;

	f.setSamplePeriod(T_s);
	f.setIntegrator();

	systems::connect(eios.output, f.input);
	systems::connect(f.output, eios.input);

	eios.setOutputValue(10);
	EXPECT_DOUBLE_EQ(1*10.0*T_s, eios.getInputValue());
	EXPECT_DOUBLE_EQ(2*10.0*T_s, eios.getInputValue());
	EXPECT_DOUBLE_EQ(3*10.0*T_s, eios.getInputValue());


	systems::ManualExecutionManager mem(1);
	f.setExecutionManager(&mem);
	mem.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 1*10.0, eios.getInputValue());
	mem.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 2*10.0, eios.getInputValue());
	mem.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 3*10.0, eios.getInputValue());
}

TEST(SystemsFirstOrderFilterTest, WithEMOnConstruction) {
	systems::ManualExecutionManager mem1(T_s);
	systems::System::defaultExecutionManager = &mem1;

	ExposedIOSystem<double> eios;
	systems::FirstOrderFilter<double> f;

	f.setSamplePeriod(T_s);
	f.setIntegrator();

	systems::connect(eios.output, f.input);
	systems::connect(f.output, eios.input);

	eios.setOutputValue(10);
	mem1.runExecutionCycle();
	EXPECT_DOUBLE_EQ(1*10.0*T_s, eios.getInputValue());
	mem1.runExecutionCycle();
	EXPECT_DOUBLE_EQ(2*10.0*T_s, eios.getInputValue());
	mem1.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s, eios.getInputValue());


	systems::ManualExecutionManager mem2(1);
	f.setExecutionManager(&mem2);
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 1*10.0, eios.getInputValue());
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 2*10.0, eios.getInputValue());
	mem2.runExecutionCycle();
	EXPECT_DOUBLE_EQ(3*10.0*T_s + 3*10.0, eios.getInputValue());
}


}
