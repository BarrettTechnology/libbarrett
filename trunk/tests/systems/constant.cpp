/*
 * constant.cpp
 *
 *  Created on: Sep 27, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems/constant.h>
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


TEST(ConstantSystemTest, OutputsConstant) {
	systems::Constant<double> consys(5.1);
	ExposedIOSystem<double> eios;
	systems::connect(consys.output, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "constant value undefined";
	EXPECT_EQ(5.1, eios.getInputValue()) << "wrong constant value given";
}


}
