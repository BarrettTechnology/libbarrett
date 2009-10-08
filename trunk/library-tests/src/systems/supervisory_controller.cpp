/*
 * supervisory_controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems.h>

#include "./exposed_io_system.h"


namespace {


// TODO(dc): actually test this
TEST(SupervisoryControllerTest, DefaultCtor) {
	Systems::ExposedIO<double> eios;
	Systems::SupervisoryController sc;

	// sc.trackReferenceSignal(eios.output);
}


}
