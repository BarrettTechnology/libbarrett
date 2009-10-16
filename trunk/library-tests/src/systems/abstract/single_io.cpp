/*
 * single_io.cpp
 *
 *  Created on: Sep 27, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/systems.h>
#include "../exposed_io_system.h"


namespace {
using namespace barrett;


TEST(SingleIOTest, DefaultCtor) {
	ExposedIOSystem<double> eios;
	checkDisconnected(eios);
}

TEST(SingleIOTest, InitialValueCtor) {
	ExposedIOSystem<double> eios(-878.3);
	systems::connect(eios.output, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "input value undefined";
	EXPECT_EQ(-878.3, eios.getInputValue()) << "input has the wrong value";
}


}
