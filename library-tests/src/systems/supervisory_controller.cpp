/*
 * supervisory_controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <stdexcept>

#include <gtest/gtest.h>
#include <barrett/systems.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;


// TODO(dc): actually test this
TEST(SupervisoryControllerTest, DefaultCtor) {
	ExposedIOSystem<double> eios;
	systems::SupervisoryController sc;

	EXPECT_THROW(sc.trackReferenceSignal(eios.output), std::invalid_argument);
}


}
