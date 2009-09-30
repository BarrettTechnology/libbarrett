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


using Systems::checkConnected;
using Systems::checkNotConnected;
using Systems::checkDisconnected;


TEST(SingleIOTest, DefaultCtor) {
	Systems::ExposedIO<double> eios;
	checkDisconnected(eios);
}

TEST(SingleIOTest, InitialValueCtor) {
	Systems::ExposedIO<double> eios(-878.3);
	Systems::connect(eios.output, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "input value undefined";
	EXPECT_EQ(-878.3, eios.getInputValue()) << "input has the wrong value";
}


}
