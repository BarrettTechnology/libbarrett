/*
 * constant.cpp
 *
 *  Created on: Sep 27, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems.h>
#include "./exposed_io_system.h"


namespace {


TEST(ConstantSystemTest, OutputsConstant) {
	Systems::Constant<double> consys(5.1);
	Systems::ExposedIO<double> eios;
	Systems::connect(consys.output, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "constant value undefined";
	EXPECT_EQ(5.1, eios.getInputValue()) << "wrong constant value given";
}


}
