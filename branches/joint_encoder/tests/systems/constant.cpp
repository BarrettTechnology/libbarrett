/*
 * constant.cpp
 *
 *  Created on: Sep 27, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems/constant.h>
#include <barrett/systems/manual_execution_manager.h>
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


TEST(ConstantSystemTest, OutputsConstant) {
	systems::ManualExecutionManager mem;
	systems::Constant<double> conSys(5.1);

	ExposedIOSystem<double> eios;
	mem.startManaging(eios);

	systems::connect(conSys.output, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "constant value undefined";
	EXPECT_EQ(5.1, eios.getInputValue()) << "wrong constant value given";
}


}
